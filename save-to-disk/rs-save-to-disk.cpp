// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams
#include <boost/thread.hpp>
#include <time.h>
//#include <librealsense2/rs_types.h>

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// Helper function for writing metadata to disk as a csv file
void metadata_to_csv(const rs2::frame& frm, const std::string& filename);

std::vector<rs2::video_frame> color_frames;
std::vector<rs2::video_frame> depth_frames;

bool finish = false;
bool trhead_finished = false;

const auto CAPACITY = 1000; // allow max latency of 5 frames
rs2::frame_queue queue(CAPACITY);
rs2::syncer sync_streams(CAPACITY);


// Declare RealSense pipeline, encapsulating the actual device and sensors
rs2::pipeline g_pipe;

void thread()
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    int depth_counter = 0;
    int color_counter = 0;
    int *counter;
    while(!finish){
        rs2::frame frame;
        if (queue.poll_for_frame(&frame)){
            frame.get_data();
            std::cout << "soy un frame" << std::endl;
            if (auto vf = frame.as<rs2::video_frame>()){

                // Use the colorizer to get an rgb image for the depth stream
                if (vf.is<rs2::depth_frame>()){ 
                    vf = color_map(frame);
                    counter = &depth_counter;
                }else{
                    counter = &color_counter;
                }

                std::string str_frame_id = std::to_string(*counter);

                if(str_frame_id.size() == 1)
                    str_frame_id = "00" + str_frame_id;
                else if(str_frame_id.size() == 2)
                    str_frame_id = "0" + str_frame_id;

                // Write images to disk
                std::stringstream png_file;

                if(vf.get_profile()stream_name().find("Depth")!=std::string::npos)
                    png_file << "Depth/";
                else if(vf.get_profile()stream_name().find("Color")!=std::string::npos)
                    png_file << "Color/";

                png_file << str_frame_id << ".png";
                //png_file << "frame-" << vf.get_profile().stream_name()  << "-" << str_frame_id << ".png";
                stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                               vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
                std::cout << "Saved " << png_file.str() << std::endl;
                ++(*counter);
            }
        }
    }
    trhead_finished = true;
}

// This sample captures 30 frames and writes the last frame to disk.
// It can be useful for debugging an embedded system with no display.
int main(int argc, char * argv[]) try
{
    // Start streaming with default recommended configuration
    //rs2::pipeline_profile selection = g_pipe.start();
    //rs2::device selected_device = selection.get_device();

    rs2::context ctx;
    auto list = ctx.query_devices();
    rs2::device selected_device = list.front();
    rs2::depth_sensor depth_sensor = selected_device.first<rs2::depth_sensor>();
    rs2::sensor color_sensor;// = selected_device.first<rs2::color_sensor>();

    rs2::stream_profile color_stream;
    rs2::stream_profile depth_stream;

    int expected_width = 640;//640 
    int expected_height = 360;//480 360
    int expected_fps = 6;//60 30 15 6

    for(auto profile : depth_sensor.get_stream_profiles()){
        int width, height;
        rs2_error* error = nullptr;
        rs2_get_video_stream_resolution(profile, &width, &height, &error);
        //std::cout << profile.format() << std::endl;
        //std::cout << width << " " << height << std::endl;
        //std::cout << profile.fps() << std::endl;
        if(profile.stream_type() == RS2_STREAM_DEPTH && profile.fps() == expected_fps && 
                expected_height == height && expected_width == width){
            depth_stream = profile;
            std::cout << "DEPTH PROFILE" << std::endl;
            break;
            //std::cout << profile.format() << std::endl;
            //std::cout << profile.fps() << std::endl;
        }
    }

    for(rs2::sensor sensor : selected_device.query_sensors()){
        std::cout << sensor.get_stream_profiles()[0].stream_type() << std::endl;
        for(auto profile : sensor.get_stream_profiles()){
            int width, height;
            rs2_error* error = nullptr;
            rs2_get_video_stream_resolution(profile, &width, &height, &error);
            //std::cout << profile.format() << std::endl;
            //std::cout << width << " " << height << std::endl;
            //std::cout << profile.fps() << std::endl;
            if(profile.stream_type() == RS2_STREAM_COLOR && profile.fps() == expected_fps && 
                    profile.format()==RS2_FORMAT_RGB8 && 
                    expected_height == height && expected_width == width){
                
                color_sensor = sensor;
                color_stream = profile;
                std::cout << "COLOR PROFILE" << std::endl;
                break;
            }              
        }

    }

    depth_sensor.open(depth_stream);
    color_sensor.open(color_stream);

    depth_sensor.start(sync_streams);
    color_sensor.start(sync_streams);

    //while(std::cin.get()!= '\n');

    //rs2::sensor mSensor get_a_sensor_from_a_device(selected_device); 
    //selected_device.first<rs2::depth_sensor>().set_option(RS2_OPTION_MOTION_RANGE, 30);

    // Wait for the next set of frames from the camera. Now that autoexposure, etc.
    // has settled, we will write these to disk
    boost::thread t{thread};

    // Capture 30 frames to give autoexposure, etc. a chance to settle
    for (auto i = 0; i < 30; ++i) sync_streams.wait_for_frames();

    double time_counter = 0;
    clock_t first_time = clock();
    int NUM_SECONDS = 20;

    usleep(1000);
    std::cout << "RECORDING..." << std::endl;
    while(time_counter < (double)(NUM_SECONDS * CLOCKS_PER_SEC)){

        rs2::frameset frames = sync_streams.wait_for_frames();
        //sync_streams.poll_for_frames(&frames);
        //= g_pipe.wait_for_frames();
        //*************g_pipe.poll_for_frames(&frames);me

        queue.enqueue(frames.get_depth_frame());
        queue.enqueue(frames.get_color_frame());

         //usleep(20);
         time_counter = (double)(clock() - first_time);
    }
    std::cout << "FINISHING..." << std::endl;
    while(std::cin.get()!='\n');
    finish=true;
    while(!trhead_finished);
    t.join();


    // To stop streaming, we simply need to call the sensor's stop method
    // After returning from the call to stop(), no frames will arrive from this sensor
    depth_sensor.stop();
    color_sensor.stop();

    // To complete the stop operation, and release access of the device, we need to call close()
    //per sensor
    depth_sensor.close();
    color_sensor.close();


    return EXIT_SUCCESS;
}
catch(const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
{
    std::ofstream csv;

    csv.open(filename);

    //    std::cout << "Writing metadata to " << filename << endl;
    csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";

    // Record all the available metadata attributes
    for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
    {
        if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
        {
            csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
                << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
        }
    }

    csv.close();
}
