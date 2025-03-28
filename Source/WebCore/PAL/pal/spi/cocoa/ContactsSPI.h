/*
 * Copyright (C) 2023 Apple Inc. All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. AND ITS CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL APPLE INC. OR ITS CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

DECLARE_SYSTEM_HEADER

#if HAVE(CONTACTS)

#import <Contacts/Contacts.h>

#if USE(APPLE_INTERNAL_SDK)

#import <Contacts/CNContact_ReallyPrivate.h>
#import <Contacts/CNLabeledValue_Private.h>
#import <Contacts/CNMutablePostalAddress_Private.h>
#import <Contacts/CNPhoneNumber_Private.h>
#import <Contacts/CNPostalAddress_Private.h>

#else // USE(APPLE_INTERNAL_SDK)

@interface CNPhoneNumber ()
+ (nonnull instancetype)phoneNumberWithDigits:(nonnull NSString *)digits countryCode:(nullable NSString *)countryCode;

@property (readonly, copy, nullable) NSString *countryCode;
@property (readonly, copy, nonnull) NSString *digits;
@end

@interface CNPostalAddress ()
@property (copy, nullable) NSString *formattedAddress;
@end

NS_ASSUME_NONNULL_BEGIN
@interface CNContact ()
- (instancetype)initWithIdentifier:(NSString *)identifier;
@end

@interface CNLabeledValue ()
- (id)initWithIdentifier:(NSString *)identifier label:(NSString *)label value:(id<NSCopying>)value;
@end
NS_ASSUME_NONNULL_END

#endif // USE(APPLE_INTERNAL_SDK)
#endif // HAVE(CONTACTS)

