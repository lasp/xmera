 /*
 ISC License

 Copyright (c) 2025, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */
%module cobConverter
%{
   #include "cobConverter.h"
%}

%include "stdint.i"
%include "std_string.i"
%include <xmera/sys_model.h>
%include "swig_conly_data.i"
%include "swig_eigen.i"

%include "cobConverter.h"

%include "architecture/msgPayloadDef/CameraModelMsgPayload.h"
%include "architecture/msgPayloadDef/NavAttMsgPayload.h"
%include "architecture/msgPayloadDef/OpNavUnitVecMsgPayload.h"
%include "architecture/msgPayloadDef/OpNavCOBMsgPayload.h"
%include "architecture/msgPayloadDef/OpNavCOMMsgPayload.h"
%include "architecture/msgPayloadDef/FilterMsgPayload.h"
