/// Derived from: https://github.com/indigo-astronomy/indigo/blob/master/indigo_drivers/ccd_touptek/indigo_ccd_touptek.c
/// by Artyom Beilis (c) 2023
//
// Copyright (c) 2018 CloudMakers, s. r. o.
// All rights reserved.
//
// You can use this software under the terms of 'INDIGO Astronomy
// open-source license' (see content below)
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHORS 'AS IS' AND ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/// INDIGO Astronomy open-source license (version 2.0, December 2020)
// Copyright (c) 2020 CloudMakers, s. r. o.
// 
// Copyright (c) 2020 Rumen G.Bogdanovski
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following
// disclaimer in the documentation and/or other materials provided
// with the distribution.
// 
// 3. Redistribution in source or binary form for fee is permitted only
// as part of another different product or as product derived from this
// software.
// 
// THIS SOFTWARE IS PROVIDED BY THE AUTHORS 'AS IS' AND ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// 


#pragma once

namespace ols {
    inline void oem_to_touptek(int &vendorId,int &productId)
    {
        if(vendorId == 0x547) {
            switch (productId) {
                case 0xe077: productId= 0x11ea; break; // Meade DSI IV Color USB3
                case 0xe078: productId= 0x11eb; break; // Meade DSI IV Color USB2
                case 0xe079: productId= 0x11f6; break; // Meade DSI IV Mono  USB3
                case 0xe07a: productId= 0x11f7; break; // Meade DSI IV Mono  USB2

                case 0xe06b: productId= 0x106b; break; // Meade DSI IV Color USB3
                case 0xe075: productId= 0x1075; break; // Meade DSI IV Color USB2
                case 0xe06d: productId= 0x106d; break; // Meade DSI IV Mono  USB3
                case 0xe076: productId= 0x1076; break; // Meade DSI IV Mono  USB2

                case 0xe00b: productId= 0x11ca; break; // Meade LPI-GC Adv  USB3
                case 0xe00c: productId= 0x11cb; break; // Meade LPI-GC Adv  USB2
                case 0xe00d: productId= 0x11cc; break; // Meade LPI-GM Adv  USB3
                case 0xe00e: productId= 0x11cd; break; // Meade LPI-GM Adv  USB2

                case 0xe007: productId= 0x115a; break; // Meade LPI-GC Adv  USB3
                case 0xe008: productId= 0x115b; break; // Meade LPI-GC Adv  USB2
                case 0xe009: productId= 0x115c; break; // Meade LPI-GM Adv  USB3
                case 0xe00a: productId= 0x115d; break; // Meade LPI-GM Adv  USB2
            }
        }
        else if(vendorId == 0x549) {
            if(productId == 0xe003) { // Meade LPI-GC
                vendorId = 0x0547;
                productId = 0x1003;
            }
            else if(productId == 0xe004) { // Meade LPI-GM
                vendorId = 0x0547;
                productId = 0x1004;
            }
        }
    }
} // namespace
