/**************************************************************************************************
 *
 * Copyright (c) 2019-2024 Axera Semiconductor Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Axera Semiconductor Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Axera Semiconductor Co., Ltd.
 *
 **************************************************************************************************/

#include "sample_engine.h"

#include "detection.hpp"

#include "string.h"
#include "stdio.h"
#include "vector"
#include "fstream"

#include "ax_engine_api.h"

#define AX_CMM_ALIGN_SIZE 128
#define ALIGN_UP(x, align) (((x) + ((align)-1)) & ~((align)-1))

const char *AX_CMM_SESSION_NAME = "npu";

typedef enum
{
    AX_ENGINE_ABST_DEFAULT = 0,
    AX_ENGINE_ABST_CACHED = 1,
} AX_ENGINE_ALLOC_BUFFER_STRATEGY_T;

typedef std::pair<AX_ENGINE_ALLOC_BUFFER_STRATEGY_T, AX_ENGINE_ALLOC_BUFFER_STRATEGY_T> INPUT_OUTPUT_ALLOC_STRATEGY;

static bool read_file(const std::string &path, std::vector<char> &data)
{
    std::fstream fs(path, std::ios::in | std::ios::binary);

    if (!fs.is_open())
    {
        return false;
    }

    fs.seekg(std::ios::end);
    auto fs_end = fs.tellg();
    fs.seekg(std::ios::beg);
    auto fs_beg = fs.tellg();

    auto file_size = static_cast<size_t>(fs_end - fs_beg);
    auto vector_size = data.size();

    data.reserve(vector_size + file_size);
    data.insert(data.end(), std::istreambuf_iterator<char>(fs), std::istreambuf_iterator<char>());

    fs.close();

    return true;
}

static void free_io_index(AX_ENGINE_IO_BUFFER_T *io_buf, int index)
{
    for (int i = 0; i < index; ++i)
    {
        AX_ENGINE_IO_BUFFER_T *pBuf = io_buf + i;
        AX_SYS_MemFree(pBuf->phyAddr, pBuf->pVirAddr);
    }
}

static void free_io(AX_ENGINE_IO_T *io)
{
    for (size_t j = 0; j < io->nInputSize; ++j)
    {
        AX_ENGINE_IO_BUFFER_T *pBuf = io->pInputs + j;
        AX_SYS_MemFree(pBuf->phyAddr, pBuf->pVirAddr);
    }
    for (size_t j = 0; j < io->nOutputSize; ++j)
    {
        AX_ENGINE_IO_BUFFER_T *pBuf = io->pOutputs + j;
        AX_SYS_MemFree(pBuf->phyAddr, pBuf->pVirAddr);
    }
    delete[] io->pInputs;
    delete[] io->pOutputs;
}

static inline int prepare_io(AX_ENGINE_IO_INFO_T *info, AX_ENGINE_IO_T *io_data, INPUT_OUTPUT_ALLOC_STRATEGY strategy)
{
    memset(io_data, 0, sizeof(*io_data));
    io_data->pInputs = new AX_ENGINE_IO_BUFFER_T[info->nInputSize];
    io_data->nInputSize = info->nInputSize;

    auto ret = 0;
    for (AX_U32 i = 0; i < info->nInputSize; ++i)
    {
        auto meta = info->pInputs[i];
        auto buffer = &io_data->pInputs[i];
        if (strategy.first == AX_ENGINE_ABST_CACHED)
        {
            ret = AX_SYS_MemAllocCached((AX_U64 *)(&buffer->phyAddr), &buffer->pVirAddr, meta.nSize, AX_CMM_ALIGN_SIZE, (const AX_S8 *)(AX_CMM_SESSION_NAME));
        }
        else
        {
            ret = AX_SYS_MemAlloc((AX_U64 *)(&buffer->phyAddr), &buffer->pVirAddr, meta.nSize, AX_CMM_ALIGN_SIZE, (const AX_S8 *)(AX_CMM_SESSION_NAME));
        }

        if (ret != 0)
        {
            free_io_index(io_data->pInputs, i);
            fprintf(stderr, "Allocate input{%d} { phy: %p, vir: %p, size: %lu Bytes }. fail \n", i, (void *)buffer->phyAddr, buffer->pVirAddr, (long)meta.nSize);
            return ret;
        }
        // fprintf(stderr, "Allocate input{%d} { phy: %p, vir: %p, size: %lu Bytes }. \n", i, (void*)buffer->phyAddr, buffer->pVirAddr, (long)meta.nSize);
    }

    io_data->pOutputs = new AX_ENGINE_IO_BUFFER_T[info->nOutputSize];
    io_data->nOutputSize = info->nOutputSize;
    for (AX_U32 i = 0; i < info->nOutputSize; ++i)
    {
        auto meta = info->pOutputs[i];
        auto buffer = &io_data->pOutputs[i];
        buffer->nSize = meta.nSize;
        if (strategy.second == AX_ENGINE_ABST_CACHED)
        {
            ret = AX_SYS_MemAllocCached((AX_U64 *)(&buffer->phyAddr), &buffer->pVirAddr, meta.nSize, AX_CMM_ALIGN_SIZE, (const AX_S8 *)(AX_CMM_SESSION_NAME));
        }
        else
        {
            ret = AX_SYS_MemAlloc((AX_U64 *)(&buffer->phyAddr), &buffer->pVirAddr, meta.nSize, AX_CMM_ALIGN_SIZE, (const AX_S8 *)(AX_CMM_SESSION_NAME));
        }
        if (ret != 0)
        {
            fprintf(stderr, "Allocate output{%d} { phy: %p, vir: %p, size: %lu Bytes }. fail \n", i, (void *)buffer->phyAddr, buffer->pVirAddr, (long)meta.nSize);
            free_io_index(io_data->pInputs, io_data->nInputSize);
            free_io_index(io_data->pOutputs, i);
            return ret;
        }
        // fprintf(stderr, "Allocate output{%d} { phy: %p, vir: %p, size: %lu Bytes }.\n", i, (void*)buffer->phyAddr, buffer->pVirAddr, (long)meta.nSize);
    }

    return 0;
}

cv::Mat BGR2YUV_NV12(const cv::Mat &src) {
    auto src_h = src.rows;
    auto src_w = src.cols;
    cv::Mat dst(src_h * 1.5, src_w, CV_8UC1);
    cv::cvtColor(src, dst, cv::COLOR_BGR2YUV_I420);  // I420: YYYY...UU...VV...
 
    auto n_y = src_h * src_w;
    auto n_uv = n_y / 2;
    auto n_u = n_y / 4;
    std::vector<uint8_t> uv(n_uv);
    std::copy(dst.data+n_y, dst.data+n_y+n_uv, uv.data());
    for (auto i = 0; i < n_u; i++) {
        dst.data[n_y + 2*i] = uv[i];            // U
        dst.data[n_y + 2*i + 1] = uv[n_u + i];  // V
    }
    return dst;
}

static void post_process(AX_ENGINE_IO_INFO_T *io_info, AX_ENGINE_IO_T *io_data, AX_VIDEO_FRAME_T* depth_map)
{
    auto& output = io_data->pOutputs[0];
    auto& info = io_info->pOutputs[0];

    cv::Mat feature(info.pShape[2], info.pShape[3], CV_32FC1, output.pVirAddr);

    double minVal, maxVal;
    cv::minMaxLoc(feature, &minVal, &maxVal);

    feature -= minVal;
    feature /= (maxVal - minVal);
    // feature = 1.f - feature;
    feature *= 255;

    feature.convertTo(feature, CV_8UC1);

    cv::Mat dst(info.pShape[2], info.pShape[3], CV_8UC3);
    cv::applyColorMap(feature, dst, cv::ColormapTypes::COLORMAP_MAGMA);

    float ratio = 0.3f;
    int dst_w = ALIGN_UP((int)(ratio * depth_map->u32Width), 16);
    int dst_h = dst_w * 1.0f / dst.cols * dst.rows;
    cv::resize(dst, dst, cv::Size(dst_w, dst_h));


    cv::Mat nv12 = BGR2YUV_NV12(dst);

    uchar* depth_virt = (uchar*)depth_map->u64VirAddr[0];
    uchar* depth_y = depth_virt;
    uchar* depth_uv = depth_virt + depth_map->u32Width * depth_map->u32Height;
    uchar* nv12_y = nv12.data;
    uchar* nv12_uv = nv12.data + dst.cols * dst.rows;
    for (int i = 0; i < dst.rows; i++)
    {
        memcpy(depth_y + i * depth_map->u32Width, nv12_y + i * nv12.cols, nv12.cols);
    }
    for (int i = 0; i < dst.rows / 2; i++)
    {
        memcpy(depth_uv + i * depth_map->u32Width, nv12_uv + i * nv12.cols, nv12.cols);
    }
}

struct ax_runner_ax650_handle_t
{
    AX_ENGINE_HANDLE handle;
    AX_ENGINE_IO_INFO_T *io_info;
    AX_ENGINE_IO_T io_data;

    int algo_width, algo_height;
    int algo_colorformat;
};

static struct ax_runner_ax650_handle_t *m_handle = nullptr;

AX_S32 SAMPLE_ENGINE_Load(AX_CHAR *model_file)
{
    if (m_handle)
    {
        return 0;
    }
    m_handle = new ax_runner_ax650_handle_t;

    // 1. init engine(vin has init)
    // AX_ENGINE_NPU_ATTR_T npu_attr;
    // memset(&npu_attr, 0, sizeof(npu_attr));
    // npu_attr.eHardMode = AX_ENGINE_VIRTUAL_NPU_DISABLE;
    // auto ret = AX_ENGINE_Init(&npu_attr);
    // if (0 != ret)
    // {
    //     return ret;
    // }

    // 2. load model
    std::vector<char> model_buffer;
    if (!read_file(model_file, model_buffer))
    {
        fprintf(stderr, "Read Run-Joint model(%s) file failed.\n", model_file);
        return -1;
    }

    // 3. create handle

    auto ret = AX_ENGINE_CreateHandle(&m_handle->handle, model_buffer.data(), model_buffer.size());
    if (0 != ret)
    {
        return ret;
    }
    // fprintf(stdout, "Engine creating handle is done.\n");

    // 4. create context
    ret = AX_ENGINE_CreateContext(m_handle->handle);
    if (0 != ret)
    {
        return ret;
    }
    // fprintf(stdout, "Engine creating context is done.\n");

    // 5. set io

    ret = AX_ENGINE_GetIOInfo(m_handle->handle, &m_handle->io_info);
    if (0 != ret)
    {
        return ret;
    }
    // fprintf(stdout, "Engine get io info is done. \n");

    // 6. alloc io
    ret = prepare_io(m_handle->io_info, &m_handle->io_data, std::make_pair(AX_ENGINE_ABST_DEFAULT, AX_ENGINE_ABST_CACHED));
    if (0 != ret)
    {
        return ret;
    }
    // fprintf(stdout, "Engine alloc io is done. \n");

    m_handle->algo_width = m_handle->io_info->pInputs[0].pShape[2];

    switch (m_handle->io_info->pInputs[0].pExtraMeta->eColorSpace)
    {
    case AX_ENGINE_CS_NV12:
        m_handle->algo_colorformat = (int)AX_FORMAT_YUV420_SEMIPLANAR;
        m_handle->algo_height = m_handle->io_info->pInputs[0].pShape[1] / 1.5;
        // printf("NV12 MODEL\n");
        break;
    case AX_ENGINE_CS_RGB:
        m_handle->algo_colorformat = (int)AX_FORMAT_RGB888;
        m_handle->algo_height = m_handle->io_info->pInputs[0].pShape[1];
        // printf("RGB MODEL\n");
        break;
    case AX_ENGINE_CS_BGR:
        m_handle->algo_colorformat = (int)AX_FORMAT_BGR888;
        m_handle->algo_height = m_handle->io_info->pInputs[0].pShape[1];
        // printf("BGR MODEL\n");
        break;
    default:
        printf("now just only support NV12/RGB/BGR input format,you can modify by yourself");
        return -1;
    }

    return ret;
}

AX_S32 SAMPLE_ENGINE_Release()
{
    if (m_handle && m_handle->handle)
    {
        free_io(&m_handle->io_data);
        AX_ENGINE_DestroyHandle(m_handle->handle);
    }
    delete m_handle;
    m_handle = nullptr;
    return 0;
}

AX_S32 SAMPLE_ENGINE_Inference(AX_VIDEO_FRAME_T *pFrame, AX_VIDEO_FRAME_T *pDepthMap)
{
    unsigned char *dst = (unsigned char *)m_handle->io_data.pInputs[0].pVirAddr;
    unsigned char *src = (unsigned char *)pFrame->u64VirAddr[0];

    memcpy(dst, src, pFrame->u32FrameSize);
    // memset(dst, 0, pFrame->u32FrameSize);

    // static int frame_count = 50;
    // static int i = 0;
    // if (i < frame_count)
    // {
    //     char filename[64] = {0};
    //     sprintf(filename, "input_%d.bin", i);
    //     FILE* fp = fopen(filename, "wb");
    //     fwrite(src, pFrame->u32FrameSize, 1, fp);
    //     fclose(fp);
    //     i++;
    // }
    
    AX_S32 ret = AX_ENGINE_RunSync(m_handle->handle, &m_handle->io_data);
    if (ret)
    {
        fprintf(stderr, "AX_ENGINE_RunSync 0x%x\n", ret);
        return -1;
    }

    post_process(m_handle->io_info, &m_handle->io_data, pDepthMap);

    return 0;
}
