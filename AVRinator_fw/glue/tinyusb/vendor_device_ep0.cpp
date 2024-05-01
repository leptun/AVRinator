/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if (CFG_TUD_ENABLED && CFG_TUD_VENDOR_EP0)

#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include "vendor_device_ep0.hpp"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
typedef struct
{
  uint8_t itf_num;

  /*------------- From this point, data is not cleared by bus reset -------------*/
} vendord_ep0_interface_t;

CFG_TUD_MEM_SECTION tu_static vendord_ep0_interface_t _vendord_ep0_itf[CFG_TUD_VENDOR_EP0];


//--------------------------------------------------------------------+
// USBD Driver API
//--------------------------------------------------------------------+
void vendord_ep0_init(void) {
  tu_memclr(_vendord_ep0_itf, sizeof(_vendord_ep0_itf));
}

bool vendord_ep0_deinit(void) {
  return true;
}

void vendord_ep0_reset(uint8_t rhport)
{
  (void) rhport;
}

uint16_t vendord_ep0_open(uint8_t rhport, tusb_desc_interface_t const * desc_itf, uint16_t max_len)
{
  TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == desc_itf->bInterfaceClass, 0);

  uint8_t const * p_desc = tu_desc_next(desc_itf);
  uint8_t const * desc_end = p_desc + max_len;

  // Find available interface
  vendord_ep0_interface_t* p_vendor_ep0 = NULL;
  for(uint8_t i=0; i<CFG_TUD_VENDOR_EP0; i++)
  {
      p_vendor_ep0 = &_vendord_ep0_itf[i];
      break;
  }
  TU_VERIFY(p_vendor_ep0, 0);

  p_vendor_ep0->itf_num = desc_itf->bInterfaceNumber;
  if (desc_itf->bNumEndpoints)
  {
    // skip non-endpoint descriptors
    while ( (TUSB_DESC_ENDPOINT != tu_desc_type(p_desc)) && (p_desc < desc_end) )
    {
      p_desc = tu_desc_next(p_desc);
    }
  }

  return (uint16_t) ((uintptr_t) p_desc - (uintptr_t) desc_itf);
}

#endif
