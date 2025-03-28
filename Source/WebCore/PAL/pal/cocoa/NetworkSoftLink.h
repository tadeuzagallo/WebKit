/*
 * Copyright (C) 2024 Apple Inc. All rights reserved.
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

#if HAVE(WEB_TRANSPORT)

#import <pal/spi/cocoa/NetworkSPI.h>
#import <wtf/SoftLinking.h>

SOFT_LINK_FRAMEWORK_FOR_HEADER(PAL, Network)

// FIXME: remove this soft linking once rdar://141009498 is available on all tested OS builds.
SOFT_LINK_FUNCTION_FOR_HEADER(PAL, Network, nw_webtransport_options_add_connect_request_header, void, (nw_protocol_options_t options, const char* name, const char* value), (options, name, value))
#define nw_webtransport_options_add_connect_request_header PAL::softLink_Network_nw_webtransport_options_add_connect_request_header

// FIXME: remove this soft linking once rdar://141715299 is available on all tested OS builds.
SOFT_LINK_FUNCTION_FOR_HEADER(PAL, Network, nw_webtransport_metadata_get_session_error_code, uint32_t, (nw_protocol_metadata_t metadata), (metadata))
#define nw_webtransport_metadata_get_session_error_code PAL::softLink_Network_nw_webtransport_metadata_get_session_error_code

// FIXME: remove this soft linking once rdar://141715299 is available on all tested OS builds.
SOFT_LINK_FUNCTION_FOR_HEADER(PAL, Network, nw_webtransport_metadata_set_session_error_code, void, (nw_protocol_metadata_t metadata, uint32_t session_error_code), (metadata, session_error_code))
#define nw_webtransport_metadata_set_session_error_code PAL::softLink_Network_nw_webtransport_metadata_set_session_error_code

// FIXME: remove this soft linking once rdar://141715299 is available on all tested OS builds.
SOFT_LINK_FUNCTION_FOR_HEADER(PAL, Network, nw_webtransport_metadata_get_session_error_message, const char*, (nw_protocol_metadata_t metadata), (metadata))
#define nw_webtransport_metadata_get_session_error_message PAL::softLink_Network_nw_webtransport_metadata_get_session_error_message

// FIXME: remove this soft linking once rdar://141715299 is available on all tested OS builds.
SOFT_LINK_FUNCTION_FOR_HEADER(PAL, Network, nw_webtransport_metadata_set_session_error_message, void, (nw_protocol_metadata_t metadata, const char* session_error_message), (metadata, session_error_message))
#define nw_webtransport_metadata_set_session_error_message PAL::softLink_Network_nw_webtransport_metadata_set_session_error_message

#endif // HAVE(WEB_TRANSPORT)
