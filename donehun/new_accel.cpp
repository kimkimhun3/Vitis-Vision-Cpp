// accel_equalizeHist_singleport_tworeads.cpp
// Single AXI input; reads input twice; calls 3-arg xf::cv::equalizeHist(in0, in1, out)
// Touches Y only (host should pass NV12 Y, and rebuild NV12 with original UV)

#ifndef _XF_HIST_EQUALIZE_NV12_CONFIG_H_
#define _XF_HIST_EQUALIZE_NV12_CONFIG_H_

#include "hls_stream.h"
#include "ap_int.h"
#include "common/xf_common.hpp"
#include "common/xf_utility.hpp"
#include "imgproc/xf_hist_equalize.hpp"

// ----- Max canvas (runtime rows/cols must be <= these) -----
#define WIDTH_4k   3840
#define HEIGHT_4k  2160
#define WIDTH_2k   1920
#define HEIGHT_2k  1080

// ----- Parallelism / pixel type -----
#define NPPCX             XF_NPPC1      // For 4K60, re-synthesize with XF_NPPC8 or XF_NPPC16
#define IN_TYPE           XF_8UC1
#define OUT_TYPE          XF_8UC1

// ----- Internal stream depths (tune as needed) -----
#define XF_CV_DEPTH_IN_1  2
#define XF_CV_DEPTH_IN_2  2
#define XF_CV_DEPTH_OUT   2

// ----- Memory options -----
#define XF_USE_URAM       0             // Set 1 if you want URAM for large FIFOs

// ----- AXI widths (bits) -----
#define INPUT_PTR_WIDTH    256
#define OUTPUT_PTR_WIDTH   256

#endif // _XF_HIST_EQUALIZE_NV12_CONFIG_H_

extern "C" {
void equalizeHist_accel(ap_uint<INPUT_PTR_WIDTH>*  img_y,     // single input port
                        ap_uint<OUTPUT_PTR_WIDTH>* img_y_out, // output port
                        int rows,
                        int cols) {
#pragma HLS INTERFACE m_axi     port=img_y     offset=slave bundle=gmem1
#pragma HLS INTERFACE m_axi     port=img_y_out offset=slave bundle=gmem2

#pragma HLS INTERFACE s_axilite port=rows
#pragma HLS INTERFACE s_axilite port=cols
#pragma HLS INTERFACE s_axilite port=return

    // Mats (runtime rows/cols bounded by HEIGHT_4k/WIDTH_4k)
    xf::cv::Mat<IN_TYPE,  HEIGHT_4k, WIDTH_4k, NPPCX, XF_CV_DEPTH_IN_1>  in_mat_pass1(rows, cols);
    xf::cv::Mat<IN_TYPE,  HEIGHT_4k, WIDTH_4k, NPPCX, XF_CV_DEPTH_IN_2>  in_mat_pass2(rows, cols);
    xf::cv::Mat<OUT_TYPE, HEIGHT_4k, WIDTH_4k, NPPCX, XF_CV_DEPTH_OUT>   out_mat(rows, cols);

#pragma HLS DATAFLOW

    // Read the SAME Y plane twice from the SAME AXI pointer (two streaming passes)
    xf::cv::Array2xfMat<INPUT_PTR_WIDTH, IN_TYPE, HEIGHT_4k, WIDTH_4k, NPPCX, XF_CV_DEPTH_IN_1>(img_y, in_mat_pass1);
    xf::cv::Array2xfMat<INPUT_PTR_WIDTH, IN_TYPE, HEIGHT_4k, WIDTH_4k, NPPCX, XF_CV_DEPTH_IN_2>(img_y, in_mat_pass2);

    // 3-argument Mat API: build LUT from in_mat_pass1, apply to in_mat_pass2
    xf::cv::equalizeHist<IN_TYPE, HEIGHT_4k, WIDTH_4k, NPPCX, XF_USE_URAM, XF_CV_DEPTH_IN_1, XF_CV_DEPTH_IN_2, XF_CV_DEPTH_OUT>(in_mat_pass1, in_mat_pass2, out_mat);

    // Write back equalized Y
    xf::cv::xfMat2Array<OUTPUT_PTR_WIDTH, OUT_TYPE, HEIGHT_4k, WIDTH_4k, NPPCX, XF_CV_DEPTH_OUT>(out_mat, img_y_out);
}
}
