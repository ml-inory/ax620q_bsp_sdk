/**************************************************************************************************
 *
 * Copyright (c) 2019-2024 Axera Semiconductor Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Axera Semiconductor Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Axera Semiconductor Co., Ltd.
 *
 **************************************************************************************************/

#ifndef _SAMPLE_ENGINE_H_
#define _SAMPLE_ENGINE_H_
#include "ax_sys_api.h"
#include "ax_ivps_api.h"

#define SAMPLE_ENGINE_MODEL_FILE "./models/depth_anything.axmodel"
#define INFER_WIDTH     384
#define INFER_HEIGHT    256
#define INFER_FORMAT    AX_FORMAT_YUV420_SEMIPLANAR

#ifdef __cplusplus
extern "C"
{
#endif
    AX_S32 SAMPLE_ENGINE_Load(AX_CHAR *model_file);
    AX_S32 SAMPLE_ENGINE_Release();
    AX_S32 SAMPLE_ENGINE_Inference(AX_VIDEO_FRAME_T *pFrame, AX_VIDEO_FRAME_T *pDepthMap);
#ifdef __cplusplus
}
#endif

#endif