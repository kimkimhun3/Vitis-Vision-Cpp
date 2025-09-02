/*
 * Benchmark OpenCV vs FPGA equalizeHist on NV12 Y-channel
 */

#include "common/xf_headers.hpp"
#include "xf_hist_equalize_tb_config.h"
#include "xcl2.hpp"
#include <chrono>

int main(int argc, char** argv) {
    if (argc != 2) {
        fprintf(stderr, "Invalid Number of Arguments!\nUsage:\n");
        fprintf(stderr, "<Executable Name> <input image path> \n");
        return -1;
    }

    // -------------------- Load Image --------------------
    cv::Mat bgr = cv::imread(argv[1]);
    if (bgr.empty()) {
        fprintf(stderr, "Cannot open image\n");
        return -1;
    }

    int height = bgr.rows;
    int width  = bgr.cols;

    std::cout << "Input image height : " << height << std::endl;
    std::cout << "Input image width  : " << width  << std::endl;

    // Convert BGR to YUV I420 (YUV420 planar)
    cv::Mat yuv;
    cv::cvtColor(bgr, yuv, cv::COLOR_BGR2YUV_I420);

    // Extract Y plane (first height*width bytes)
    cv::Mat y_plane(height, width, CV_8UC1, yuv.data);

    // Output buffers
    cv::Mat y_ocv(height, width, CV_8UC1);
    cv::Mat y_fpga(height, width, CV_8UC1);
    cv::Mat diff(height, width, CV_8UC1);

    // -------------------- OpenCV Software --------------------
    auto t1 = std::chrono::high_resolution_clock::now();
    cv::equalizeHist(y_plane, y_ocv);
    auto t2 = std::chrono::high_resolution_clock::now();
    double ocv_time = std::chrono::duration<double, std::milli>(t2 - t1).count();
    std::cout << "OpenCV equalizeHist time: " << ocv_time << " ms" << std::endl;

    // -------------------- FPGA OpenCL Setup --------------------
    std::vector<cl::Device> devices = xcl::get_xil_devices();
    cl::Device device = devices[0];
    cl::Context context(device);
    cl::CommandQueue q(context, device, CL_QUEUE_PROFILING_ENABLE);

    std::string device_name = device.getInfo<CL_DEVICE_NAME>();
    std::string binaryFile = xcl::find_binary_file(device_name, "krnl_hist_equalize");
    cl::Program::Binaries bins = xcl::import_binary_file(binaryFile);
    devices.resize(1);
    cl::Program program(context, devices, bins);
    cl::Kernel krnl(program, "equalizeHist_accel");

    cl::Buffer imageToDevice1(context, CL_MEM_READ_ONLY, height * width);
    cl::Buffer imageToDevice2(context, CL_MEM_READ_ONLY, height * width);
    cl::Buffer imageFromDevice(context, CL_MEM_WRITE_ONLY, height * width);

    // Set arguments
    krnl.setArg(0, imageToDevice1);
    krnl.setArg(1, imageToDevice2);
    krnl.setArg(2, imageFromDevice);
    krnl.setArg(3, height);
    krnl.setArg(4, width);

    // -------------------- FPGA Run --------------------
    q.enqueueWriteBuffer(imageToDevice1, CL_TRUE, 0, height * width, y_plane.data);
    q.enqueueWriteBuffer(imageToDevice2, CL_TRUE, 0, height * width, y_plane.data);

    cl::Event event_sp;
    q.enqueueTask(krnl, NULL, &event_sp);
    clWaitForEvents(1, (const cl_event*)&event_sp);

    cl_ulong start, end;
    event_sp.getProfilingInfo(CL_PROFILING_COMMAND_START, &start);
    event_sp.getProfilingInfo(CL_PROFILING_COMMAND_END, &end);
    double fpga_time = (end - start) / 1e6; // ms
    std::cout << "FPGA equalizeHist time: " << fpga_time << " ms" << std::endl;

    q.enqueueReadBuffer(imageFromDevice, CL_TRUE, 0, height * width, y_fpga.data);
    q.finish();

    // -------------------- Compare Results --------------------
    cv::absdiff(y_ocv, y_fpga, diff);

    float err_per;
    xf::cv::analyzeDiff(diff, 1, err_per);

    if (err_per > 0.0f) {
        std::cerr << "ERROR: Results mismatch, error = " << err_per << "%" << std::endl;
    } else {
        std::cout << "Results match!" << std::endl;
    }

    // -------------------- Save Debug Images --------------------
    cv::imwrite("input_y.jpg", y_plane);
    cv::imwrite("out_ocv_y.jpg", y_ocv);
    cv::imwrite("out_fpga_y.jpg", y_fpga);
    cv::imwrite("out_diff_y.jpg", diff);

    return 0;
}
