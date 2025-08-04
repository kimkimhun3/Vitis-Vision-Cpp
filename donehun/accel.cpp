#ifndef _XF_HIST_EQUALIZE_NV12_CONFIG_H_
#define _XF_HIST_EQUALIZE_NV12_CONFIG_H_

#include "hls_stream.h"
#include "ap_int.h"
#include "common/xf_common.hpp"
#include "common/xf_utility.hpp"
#include "imgproc/xf_hist_equalize.hpp"

#define WIDTH_4k 3840
#define HEIGHT_4k 2160
#define WIDTH_2k 1920
#define HEIGHT_2k 1080

#define NPPCX XF_NPPC1

#define IN_TYPE XF_8UC1
#define OUT_TYPE XF_8UC1

#define XF_CV_DEPTH_IN_1 2
#define XF_CV_DEPTH_IN_2 2
#define XF_CV_DEPTH_OUT 2

#define XF_USE_URAM 0

#define CV_IN_TYPE CV_8UC1
#define CV_OUT_TYPE CV_8UC1

#define INPUT_PTR_WIDTH 256
#define OUTPUT_PTR_WIDTH 256

#endif // _XF_HIST_EQUALIZE_NV12_CONFIG_H_


extern "C" {
void equalizeHist_accel(ap_uint<INPUT_PTR_WIDTH>* img_y_in,
                        ap_uint<INPUT_PTR_WIDTH>* img_y_ref,
                        ap_uint<OUTPUT_PTR_WIDTH>* img_y_out,
                        int rows,
                        int cols) {
#pragma HLS INTERFACE m_axi     port=img_y_in  offset=slave bundle=gmem1
#pragma HLS INTERFACE m_axi     port=img_y_ref offset=slave bundle=gmem2
#pragma HLS INTERFACE m_axi     port=img_y_out offset=slave bundle=gmem3

#pragma HLS INTERFACE s_axilite port=rows       
#pragma HLS INTERFACE s_axilite port=cols       
#pragma HLS INTERFACE s_axilite port=return

    xf::cv::Mat<IN_TYPE, HEIGHT_4k, WIDTH_4k, NPPCX, XF_CV_DEPTH_IN_1> in_mat(rows, cols);
    xf::cv::Mat<IN_TYPE, HEIGHT_4k, WIDTH_4k, NPPCX, XF_CV_DEPTH_IN_2> in_mat_ref(rows, cols);
    xf::cv::Mat<OUT_TYPE, HEIGHT_4k, WIDTH_4k, NPPCX, XF_CV_DEPTH_OUT> out_mat(rows, cols);

#pragma HLS DATAFLOW

    xf::cv::Array2xfMat<INPUT_PTR_WIDTH, IN_TYPE, HEIGHT_4k, WIDTH_4k, NPPCX, XF_CV_DEPTH_IN_1>(img_y_in, in_mat);
    xf::cv::Array2xfMat<INPUT_PTR_WIDTH, IN_TYPE, HEIGHT_4k, WIDTH_4k, NPPCX, XF_CV_DEPTH_IN_2>(img_y_ref, in_mat_ref);

    xf::cv::equalizeHist<IN_TYPE, HEIGHT_4k, WIDTH_4k, NPPCX, XF_USE_URAM, XF_CV_DEPTH_IN_1, XF_CV_DEPTH_IN_2, XF_CV_DEPTH_OUT>(in_mat, in_mat_ref, out_mat);

    xf::cv::xfMat2Array<OUTPUT_PTR_WIDTH, OUT_TYPE, HEIGHT_4k, WIDTH_4k, NPPCX, XF_CV_DEPTH_OUT>(out_mat, img_y_out);
}
}
