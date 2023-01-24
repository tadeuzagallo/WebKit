/*
 * Copyright (c) 2023 Apple Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL APPLE INC. OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"
#include "GlobalVariableRewriter.h"

#include "AST.h"
#include "ASTVisitor.h"
#include "CallGraph.h"
#include <wtf/HashSet.h>

namespace WGSL {

class RewriteGlobalVariables : public AST::Visitor {
public:
    RewriteGlobalVariables(CallGraph& callGraph)
        : AST::Visitor()
        , m_callGraph(callGraph)
        , m_emptySourceSpan(0, 0, 0, 0)
    {
    }

    void run();

    void visit(AST::FunctionDecl&) override;
    void visit(AST::VariableDecl&) override;
    void visit(AST::IdentifierExpression&) override;

private:
    void def(const String&);
    void read(const String&);
    bool isGlobal(const String&);

    void collectGlobals();
    void visitEntryPoint(AST::FunctionDecl&);
    void insertParameters(AST::FunctionDecl&);

    CallGraph& m_callGraph;
    HashMap<String, UniqueRef<AST::VariableDecl>> m_globals;
    HashSet<String> m_defs;
    HashSet<String> m_reads;
    SourceSpan m_emptySourceSpan;
};

void RewriteGlobalVariables::run()
{
    collectGlobals();
    for (auto& entryPoint : m_callGraph.entrypoints())
        visitEntryPoint(entryPoint.m_function);
}

void RewriteGlobalVariables::visit(AST::FunctionDecl& functionDecl)
{
    //{
        //DefinitionScope scope(m_definitions);
        //visitCallees(functionDecl);
    //}

    for (auto& parameter : functionDecl.parameters())
        def(parameter.name());

    // FIXME: detect when we shadow a global that a callee needs
    AST::Visitor::visit(functionDecl.body());
}

void RewriteGlobalVariables::visit(AST::VariableDecl& variableDecl)
{
    def(variableDecl.name());
}

template<typename NewType, typename CurrentType, typename... Args>
static void replace(CurrentType& node, Args&&... args)
{
    static_assert(sizeof(NewType) == sizeof(CurrentType));
    new (&node) NewType(node.span(), std::forward<Args>(args)...);
}

void RewriteGlobalVariables::visit(AST::IdentifierExpression& identifier)
{
    const auto& name = identifier.identifier();
    if (isGlobal(name)) {
        read(name);
        replace<AST::PointerDereference>(
            identifier,
            makeUniqueRef<AST::IdentifierExpression>(WTFMove(identifier))
        );
    }
}

void RewriteGlobalVariables::collectGlobals()
{
    auto& globalVars = m_callGraph.ast().globalVars();
    while (!globalVars.isEmpty()) {
        auto globalVar = globalVars.takeLast();
        auto result = m_globals.add(globalVar->name(), WTFMove(globalVar));
        ASSERT_UNUSED(result, result.isNewEntry);
    }
}

void RewriteGlobalVariables::visitEntryPoint(AST::FunctionDecl& functionDecl)
{
    m_reads.clear();
    m_defs.clear();

    visit(functionDecl);
    // FIXME: not all globals are arguments
    insertParameters(functionDecl);
}

//void RewriteGlobalVariables::visitCallee(AST::FunctionDecl& functionDecl)
//{
    //visit(functionDecl);
    //insertParameters();
//}

void RewriteGlobalVariables::insertParameters(AST::FunctionDecl& functionDecl)
{
    for (const auto& globalName : m_reads) {
        auto it = m_globals.find(globalName);
        RELEASE_ASSERT(it != m_globals.end());
        auto& global = it->value;
        ASSERT(global->maybeTypeDecl());
        functionDecl.parameters().append(makeUniqueRef<AST::Parameter>(
            m_emptySourceSpan,
            globalName,
            *global->maybeTypeDecl(),
            AST::Attribute::List { },
            AST::ParameterType::GlobalVariable
        ));
    }
}

void RewriteGlobalVariables::def(const String& name)
{
    m_defs.add(name);
}

void RewriteGlobalVariables::read(const String& name)
{
    m_reads.add(name);
}

bool RewriteGlobalVariables::isGlobal(const String& string)
{
    return !m_defs.contains(string) && m_globals.contains(string);
}

void rewriteGlobalVariables(CallGraph& callGraph)
{
    RewriteGlobalVariables(callGraph).run();
}

} // namespace WGSL
