/*
 * Copyright (C) 2004, 2005, 2008 Nikolas Zimmermann <zimmermann@kde.org>
 * Copyright (C) 2004, 2005, 2006, 2007 Rob Buis <buis@kde.org>
 * Copyright (C) 2007 Eric Seidel <eric@webkit.org>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public License
 * along with this library; see the file COPYING.LIB.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#include "config.h"
#include "SVGTransformable.h"

#include "AffineTransform.h"
#include "FloatConversion.h"
#include "SVGElement.h"
#include "SVGNames.h"
#include "SVGParserUtilities.h"
#include <wtf/text/StringParsingBuffer.h>
#include <wtf/text/StringView.h>

namespace WebCore {

SVGTransformable::~SVGTransformable() = default;

template<typename CharacterType> static int parseTransformParamList(StringParsingBuffer<CharacterType>& buffer, std::span<float, 6> values, int required, int optional)
{
    int optionalParams = 0, requiredParams = 0;
    
    if (!skipOptionalSVGSpaces(buffer) || *buffer != '(')
        return -1;
    
    ++buffer;
   
    skipOptionalSVGSpaces(buffer);

    while (requiredParams < required) {
        if (buffer.atEnd())
            return -1;
        auto parsedNumber = parseNumber(buffer, SuffixSkippingPolicy::DontSkip);
        if (!parsedNumber)
            return -1;
        values[requiredParams] = *parsedNumber;
        requiredParams++;
        if (requiredParams < required)
            skipOptionalSVGSpacesOrDelimiter(buffer);
    }
    if (!skipOptionalSVGSpaces(buffer))
        return -1;
    
    bool delimParsed = skipOptionalSVGSpacesOrDelimiter(buffer);

    if (buffer.atEnd())
        return -1;
    
    if (*buffer == ')') {
        // skip optionals
        ++buffer;
        if (delimParsed)
            return -1;
    } else {
        while (optionalParams < optional) {
            if (buffer.atEnd())
                return -1;
            auto parsedNumber = parseNumber(buffer, SuffixSkippingPolicy::DontSkip);
            if (!parsedNumber)
                return -1;
            values[requiredParams + optionalParams] = *parsedNumber;
            optionalParams++;
            if (optionalParams < optional)
                skipOptionalSVGSpacesOrDelimiter(buffer);
        }
        
        if (!skipOptionalSVGSpaces(buffer))
            return -1;
        
        delimParsed = skipOptionalSVGSpacesOrDelimiter(buffer);
        
        if (buffer.atEnd() || *buffer != ')' || delimParsed)
            return -1;
        ++buffer;
    }

    return requiredParams + optionalParams;
}

// These should be kept in sync with enum SVGTransformType
static constexpr std::array requiredValuesForType { 0, 6, 1, 1, 1, 1, 1 };
static constexpr std::array optionalValuesForType { 0, 0, 1, 1, 2, 0, 0 };

template<typename CharacterType> static RefPtr<SVGTransform> parseTransformGeneric(SVGTransformValue::SVGTransformType type, StringParsingBuffer<CharacterType>& buffer, RefPtr<SVGTransform> reusableTransform)
{
    ASSERT_IMPLIES(reusableTransform, type == reusableTransform->value().type());

    if (type == SVGTransformValue::SVG_TRANSFORM_UNKNOWN)
        return nullptr;

    int valueCount = 0;
    std::array<float, 6> values { 0, 0, 0, 0, 0, 0 };
    if ((valueCount = parseTransformParamList(buffer, values, requiredValuesForType[type], optionalValuesForType[type])) < 0)
        return nullptr;

    switch (type) {
    case SVGTransformValue::SVG_TRANSFORM_UNKNOWN:
        ASSERT_NOT_REACHED();
        return nullptr;

    case SVGTransformValue::SVG_TRANSFORM_SKEWX: {
        if (reusableTransform) {
            ASSERT(reusableTransform->value().type() == SVGTransformValue::SVG_TRANSFORM_SKEWX);
            reusableTransform->value().setSkewX(values[0]);
            return reusableTransform;
        }

        SVGTransformValue transform;
        transform.setSkewX(values[0]);
        return SVGTransform::create(WTFMove(transform));
    }
    case SVGTransformValue::SVG_TRANSFORM_SKEWY: {
        if (reusableTransform) {
            reusableTransform->value().setSkewY(values[0]);
            return reusableTransform;
        }

        SVGTransformValue transform;
        transform.setSkewY(values[0]);
        return SVGTransform::create(WTFMove(transform));
    }
    case SVGTransformValue::SVG_TRANSFORM_SCALE: {
        if (reusableTransform) {
            if (valueCount == 1)
                reusableTransform->value().setScale(values[0], values[0]);
            else
                reusableTransform->value().setScale(values[0], values[1]);

            return reusableTransform;
        }

        auto resultValue = [&]() {
            if (valueCount == 1) // Spec: if only one param given, assume uniform scaling
                return SVGTransformValue::scaleTransformValue({ values[0], values[0] });

            return SVGTransformValue::scaleTransformValue({ values[0], values[1] });
        };

        return SVGTransform::create(resultValue());
    }
    case SVGTransformValue::SVG_TRANSFORM_TRANSLATE: {
        if (reusableTransform) {
            if (valueCount == 1)
                reusableTransform->value().setTranslate(values[0], values[0]);
            else
                reusableTransform->value().setTranslate(values[0], values[1]);

            return reusableTransform;
        }

        if (valueCount == 1) // Spec: if only one param given, assume 2nd param to be 0
            return SVGTransform::create(SVGTransformValue::translateTransformValue({ values[0], 0 }));

        return SVGTransform::create(SVGTransformValue::translateTransformValue({ values[0], values[1] }));
    }
    case SVGTransformValue::SVG_TRANSFORM_ROTATE: {
        if (reusableTransform) {
            if (valueCount == 1)
                reusableTransform->value().setRotate(values[0], 0, 0);
            else
                reusableTransform->value().setRotate(values[0], values[1], values[2]);

            return reusableTransform;
        }

        auto resultValue = [&]() {
            if (valueCount == 1)
                return SVGTransformValue::rotateTransformValue(values[0], { });

            return SVGTransformValue::rotateTransformValue(values[0], { values[1], values[2] });
        };
        return SVGTransform::create(resultValue());
    }
    case SVGTransformValue::SVG_TRANSFORM_MATRIX: {
        if (reusableTransform) {
            reusableTransform->value().setMatrix(AffineTransform(values[0], values[1], values[2], values[3], values[4], values[5]));
            return reusableTransform;
        }

        SVGTransformValue transform;
        transform.setMatrix(AffineTransform(values[0], values[1], values[2], values[3], values[4], values[5]));
        return SVGTransform::create(transform);
    }
    }

    return nullptr;
}

RefPtr<SVGTransform> SVGTransformable::parseTransform(SVGTransformValue::SVGTransformType type, StringParsingBuffer<LChar>& buffer, RefPtr<SVGTransform> reusableTransform)
{
    return parseTransformGeneric(type, buffer, reusableTransform);
}

RefPtr<SVGTransform> SVGTransformable::parseTransform(SVGTransformValue::SVGTransformType type, StringParsingBuffer<UChar>& buffer, RefPtr<SVGTransform> reusableTransform)
{
    return parseTransformGeneric(type, buffer, reusableTransform);
}

template<typename CharacterType> static constexpr std::array<CharacterType, 5> skewXDesc  { 's', 'k', 'e', 'w', 'X' };
template<typename CharacterType> static constexpr std::array<CharacterType, 5> skewYDesc  { 's', 'k', 'e', 'w', 'Y' };
template<typename CharacterType> static constexpr std::array<CharacterType, 5> scaleDesc  { 's', 'c', 'a', 'l', 'e' };
template<typename CharacterType> static constexpr std::array<CharacterType, 9> translateDesc  { 't', 'r', 'a', 'n', 's', 'l', 'a', 't', 'e' };
template<typename CharacterType> static constexpr std::array<CharacterType, 6> rotateDesc  { 'r', 'o', 't', 'a', 't', 'e' };
template<typename CharacterType> static constexpr std::array<CharacterType, 6> matrixDesc  { 'm', 'a', 't', 'r', 'i', 'x' };

template<typename CharacterType> static std::optional<SVGTransformValue::SVGTransformType> parseTransformTypeGeneric(StringParsingBuffer<CharacterType>& buffer)
{
    if (buffer.atEnd())
        return std::nullopt;

    if (*buffer == 's') {
        if (skipCharactersExactly(buffer, std::span { skewXDesc<CharacterType> }))
            return SVGTransformValue::SVG_TRANSFORM_SKEWX;
        if (skipCharactersExactly(buffer, std::span { skewYDesc<CharacterType> }))
            return SVGTransformValue::SVG_TRANSFORM_SKEWY;
        if (skipCharactersExactly(buffer, std::span { scaleDesc<CharacterType> }))
            return SVGTransformValue::SVG_TRANSFORM_SCALE;
        return std::nullopt;
    }

    if (skipCharactersExactly(buffer, std::span { translateDesc<CharacterType> }))
        return SVGTransformValue::SVG_TRANSFORM_TRANSLATE;
    if (skipCharactersExactly(buffer, std::span { rotateDesc<CharacterType> }))
        return SVGTransformValue::SVG_TRANSFORM_ROTATE;
    if (skipCharactersExactly(buffer, std::span { matrixDesc<CharacterType> }))
        return SVGTransformValue::SVG_TRANSFORM_MATRIX;

    return std::nullopt;
}

std::optional<SVGTransformValue::SVGTransformType> SVGTransformable::parseTransformType(StringView string)
{
    return readCharactersForParsing(string, [](auto buffer) {
        return parseTransformType(buffer);
    });
}

std::optional<SVGTransformValue::SVGTransformType> SVGTransformable::parseTransformType(StringParsingBuffer<LChar>& buffer)
{
    return parseTransformTypeGeneric(buffer);
}

std::optional<SVGTransformValue::SVGTransformType> SVGTransformable::parseTransformType(StringParsingBuffer<UChar>& buffer)
{
    return parseTransformTypeGeneric(buffer);
}

}
