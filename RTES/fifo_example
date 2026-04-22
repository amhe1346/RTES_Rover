/*
 *  Real-time Transform Analysis - SCHED_FIFO
 *  Tests: 3 transforms × 3 resolutions = 9 datasets
 *  
 *  Transforms: Canny, Hough Lines, Hough Circles
 *  Resolutions: 160×120, 320×240, 640×480
 *  Scheduler: SCHED_FIFO (priority 90)
 *  Frames per test: 100
 *  Deadline: 70 ms
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <sched.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

#define MAX_FRAMES 100
#define DEADLINE_DEFAULT_US 70000LL
#define DEADLINE_640_HOUGH_LINES_US 110000LL
#define DEADLINE_640_HOUGH_CIRCLES_US 130000LL

// Transform types
typedef enum {
    TRANSFORM_CANNY = 0,
    TRANSFORM_HOUGH_LINES,
    TRANSFORM_HOUGH_CIRCLES,
    TRANSFORM_COUNT
} TransformType;

// Resolution configurations
typedef struct {
    int width;
    int height;
    const char* name;
} Resolution;

Resolution resolutions[] = {
    {160, 120, "160x120"},
    {320, 240, "320x240"},
    {640, 480, "640x480"}
};
#define NUM_RESOLUTIONS 3

const char* transform_names[] = {
    "canny",
    "houghlines",
    "houghcircles"
};

// Global variables for transforms
Mat frame_mat, gray, edges, cdst, timg_grad;
vector<Vec4i> detected_lines;
vector<Vec3f> detected_circles;
int lowThreshold = 50;
int kernel_size = 3;
int ratio = 3;

static long long ts_to_us(const struct timespec *ts)
{
    return ((long long)ts->tv_sec * 1000000LL) + (ts->tv_nsec / 1000LL);
}

static long long get_deadline_us(TransformType transform, Resolution res)
{
    // Use 130ms deadline for Hough Circles at 640x480
    if(transform == TRANSFORM_HOUGH_CIRCLES && res.width == 640 && res.height == 480)
    {
        return DEADLINE_640_HOUGH_CIRCLES_US;
    }
    // Use 110ms deadline for Hough Lines at 640x480
    if(transform == TRANSFORM_HOUGH_LINES && res.width == 640 && res.height == 480)
    {
        return DEADLINE_640_HOUGH_LINES_US;
    }
    // Default 70ms deadline for all others
    return DEADLINE_DEFAULT_US;
}

void apply_canny_transform(Mat& input, Mat& output)
{
    cvtColor(input, gray, CV_RGB2GRAY);
    blur(gray, edges, Size(3, 3));
    Canny(edges, edges, lowThreshold, lowThreshold * ratio, kernel_size);
    
    output = Scalar::all(0);
    input.copyTo(output, edges);
}

void apply_hough_lines_transform(Mat& input, Mat& output)
{
    cvtColor(input, gray, CV_RGB2GRAY);
    Canny(gray, edges, 50, 150, 3);
    
    detected_lines.clear();
    HoughLinesP(edges, detected_lines, 1, CV_PI/180, 50, 50, 10);
    
    // Store edges for visualization (done after timing)
    cvtColor(edges, output, CV_GRAY2BGR);
}

void draw_hough_lines(Mat& output)
{
    for(size_t i = 0; i < detected_lines.size(); i++)
    {
        Vec4i l = detected_lines[i];
        line(output, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, CV_AA);
    }
}

void apply_hough_circles_transform(Mat& input, Mat& output)
{
    cvtColor(input, gray, CV_RGB2GRAY);
    GaussianBlur(gray, gray, Size(9, 9), 2, 2);
    
    detected_circles.clear();
    HoughCircles(gray, detected_circles, CV_HOUGH_GRADIENT, 1, gray.rows/8, 100, 50, 0, 0);
    
    // Store input for visualization (done after timing)
    output = input.clone();
}

void draw_hough_circles(Mat& output)
{
    for(size_t i = 0; i < detected_circles.size(); i++)
    {
        Point center(cvRound(detected_circles[i][0]), cvRound(detected_circles[i][1]));
        int radius = cvRound(detected_circles[i][2]);
        circle(output, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        circle(output, center, radius, Scalar(0, 0, 255), 3, 8, 0);
    }
}

void apply_transform(TransformType transform, Mat& input, Mat& output)
{
    switch(transform)
    {
        case TRANSFORM_CANNY:
            apply_canny_transform(input, output);
            break;
        case TRANSFORM_HOUGH_LINES:
            apply_hough_lines_transform(input, output);
            break;
        case TRANSFORM_HOUGH_CIRCLES:
            apply_hough_circles_transform(input, output);
            break;
        default:
            output = input.clone();
            break;
    }
}

void run_test(CvCapture* capture, TransformType transform, Resolution res)
{
    long long DEADLINE_US = get_deadline_us(transform, res);
    
    char csv_name[256];
    char window_name[256];
    
    snprintf(csv_name, sizeof(csv_name), "fifo_%s_%s.csv", 
             transform_names[transform], res.name);
    snprintf(window_name, sizeof(window_name), "FIFO - %s - %s", 
             transform_names[transform], res.name);
    
    printf("\n========================================\n");
    printf("Test: %s at %s\n", transform_names[transform], res.name);
    printf("CSV: %s\n", csv_name);
    printf("========================================\n");
    
    FILE *csv_fp = fopen(csv_name, "w");
    if(csv_fp == NULL)
    {
        printf("ERROR: could not open CSV file %s\n", csv_name);
        return;
    }
    
    // Set resolution
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, res.width);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, res.height);
    
    // Set FPS to 30 (important!)
    cvSetCaptureProperty(capture, CV_CAP_PROP_FPS, 30);
    
    // Wait a moment for camera to adjust
    usleep(500000); // 500ms
        // Verify actual settings
    double actual_fps = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
    double actual_width = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
    double actual_height = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
    printf("Camera: %.0fx%.0f @ %.1f FPS\n", actual_width, actual_height, actual_fps);
    
    namedWindow(window_name, CV_WINDOW_AUTOSIZE);
    
    int frame_count = 0;
    int miss_count = 0;
    
    struct timespec start_ts, end_ts;
    long long start_us, end_us, exec_us, prev_end_us = 0, delta_us;
    int miss;
    
    long long exec_sum_us = 0;
    long long delta_sum_us = 0;
    long long exec_max_us = 0;
    long long delta_max_us = 0;
    long long exec_min_us = LLONG_MAX;
    long long delta_min_us = LLONG_MAX;
    double exec_sum_sq = 0.0;
    double delta_sum_sq = 0.0;
    int stats_count = 0;
    
    fprintf(csv_fp, "frame,start_us,end_us,exec_us,delta_us,deadline_us,miss\n");
    
    while(frame_count < MAX_FRAMES)
    {
        clock_gettime(CLOCK_MONOTONIC, &start_ts);
        
        IplImage* frame_ipl = cvQueryFrame(capture);
        if(!frame_ipl)
        {
            printf("ERROR: frame capture failed\n");
            break;
        }
        
        // Convert to Mat
        frame_mat = cvarrToMat(frame_ipl);
        
        // Apply transform (detection only, no visualization)
        apply_transform(transform, frame_mat, timg_grad);
        
        // End timing (excludes visualization and display overhead)
        clock_gettime(CLOCK_MONOTONIC, &end_ts);
        
        // Draw visualization overlays (after timing)
        if(transform == TRANSFORM_HOUGH_LINES)
            draw_hough_lines(timg_grad);
        else if(transform == TRANSFORM_HOUGH_CIRCLES)
            draw_hough_circles(timg_grad);
        
        // Display (not counted in RT performance)
        imshow(window_name, timg_grad);
        
        start_us = ts_to_us(&start_ts);
        end_us = ts_to_us(&end_ts);
        exec_us = end_us - start_us;
        
        if(frame_count == 0)
        {
            delta_us = 0;
            miss = 0;
        }
        else
        {
            delta_us = end_us - prev_end_us;
            miss = (delta_us > DEADLINE_US) ? 1 : 0;
        }
        
        fprintf(csv_fp, "%d,%lld,%lld,%lld,%lld,%lld,%d\n",
                frame_count, start_us, end_us, exec_us, delta_us, DEADLINE_US, miss);
        
        if(frame_count > 0)
        {
            exec_sum_us += exec_us;
            delta_sum_us += delta_us;
            exec_sum_sq += (double)exec_us * (double)exec_us;
            delta_sum_sq += (double)delta_us * (double)delta_us;
            
            if(exec_us > exec_max_us)
                exec_max_us = exec_us;
            
            if(exec_us < exec_min_us)
                exec_min_us = exec_us;
            
            if(delta_us > delta_max_us)
                delta_max_us = delta_us;
            
            if(delta_us < delta_min_us)
                delta_min_us = delta_us;
            
            miss_count += miss;
            stats_count++;
        }
        
        prev_end_us = end_us;
        frame_count++;
        
        // Allow 33ms for display refresh and key check (~30 FPS)
        char q = cvWaitKey(33);
        if(q == 'q')
        {
            printf("User quit at frame %d\n", frame_count);
            break;
        }
    }
    
    // Write summary statistics
    if(stats_count > 0)
    {
        double avg_exec_us = (double)exec_sum_us / (double)stats_count;
        double avg_delta_us = (double)delta_sum_us / (double)stats_count;
        double fps = 1000000.0 / avg_delta_us;
        double miss_percent = (100.0 * (double)miss_count) / (double)stats_count;
        
        // Calculate standard deviation
        double exec_variance = (exec_sum_sq / (double)stats_count) - (avg_exec_us * avg_exec_us);
        double delta_variance = (delta_sum_sq / (double)stats_count) - (avg_delta_us * avg_delta_us);
        double exec_std = (exec_variance > 0) ? sqrt(exec_variance) : 0.0;
        double delta_std = (delta_variance > 0) ? sqrt(delta_variance) : 0.0;
        
        // Calculate jitter
        long long exec_jitter = exec_max_us - exec_min_us;
        long long delta_jitter = delta_max_us - delta_min_us;
        
        // Calculate coefficient of variation
        double exec_cv = (avg_exec_us > 0) ? (exec_std / avg_exec_us) : 0.0;
        double delta_cv = (avg_delta_us > 0) ? (delta_std / avg_delta_us) : 0.0;
        
        printf("\n----- Timing Summary (excluding frame 0) -----\n");
        printf("Frames analyzed: %d\n", stats_count);
        printf("\n--- Execution Time Statistics ---\n");
        printf("  Mean:        %8.2f μs  (%6.2f ms)\n", avg_exec_us, avg_exec_us / 1000.0);
        printf("  Std Dev:     %8.2f μs  (%6.2f ms)\n", exec_std, exec_std / 1000.0);
        printf("  Min:         %8lld μs  (%6.2f ms)\n", exec_min_us, exec_min_us / 1000.0);
        printf("  Max:         %8lld μs  (%6.2f ms)\n", exec_max_us, exec_max_us / 1000.0);
        printf("  Jitter:      %8lld μs  (%6.2f ms)\n", exec_jitter, exec_jitter / 1000.0);
        printf("  CV:          %8.4f     (%6.2f %%)\n", exec_cv, exec_cv * 100.0);
        
        printf("\n--- Frame Rate Statistics ---\n");
        printf("  Average FPS: %8.2f\n", fps);
        printf("  Delta Mean:  %8.2f μs  (%6.2f ms)\n", avg_delta_us, avg_delta_us / 1000.0);
        printf("  Delta Std:   %8.2f μs  (%6.2f ms)\n", delta_std, delta_std / 1000.0);
        printf("  Delta Jitter:%8lld μs  (%6.2f ms)\n", delta_jitter, delta_jitter / 1000.0);
        printf("  Delta CV:    %8.4f     (%6.2f %%)\n", delta_cv, delta_cv * 100.0);
        
        printf("\n--- Deadline Analysis ---\n");
        printf("  Deadline:    %8lld μs  (%6.2f ms)\n", DEADLINE_US, DEADLINE_US / 1000.0);
        printf("  Misses:      %8d / %d\n", miss_count, stats_count);
        printf("  Miss %%:      %8.2f %%\n", miss_percent);
        
        if(miss_percent < 1.0)
            printf("  ✓ Excellent real-time performance\n");
        else if(miss_percent < 5.0)
            printf("  ✓ Good real-time performance\n");
        else if(miss_percent < 10.0)
            printf("  ⚠ Marginal real-time performance\n");
        else
            printf("  ✗ Poor real-time performance\n");
        
        printf("----------------------------------------------\n");
        
        fprintf(csv_fp, "\n");
        fprintf(csv_fp, "# Summary (excluding frame 0)\n");
        fprintf(csv_fp, "# Frames analyzed,%d\n", stats_count);
        fprintf(csv_fp, "# Average exec time us,%.3f\n", avg_exec_us);
        fprintf(csv_fp, "# Exec std dev us,%.3f\n", exec_std);
        fprintf(csv_fp, "# Min exec time us,%lld\n", exec_min_us);
        fprintf(csv_fp, "# Max exec time us,%lld\n", exec_max_us);
        fprintf(csv_fp, "# Exec jitter us,%lld\n", exec_jitter);
        fprintf(csv_fp, "# Exec CV,%.6f\n", exec_cv);
        fprintf(csv_fp, "# Average frame delta us,%.3f\n", avg_delta_us);
        fprintf(csv_fp, "# Delta std dev us,%.3f\n", delta_std);
        fprintf(csv_fp, "# Min frame delta us,%lld\n", delta_min_us);
        fprintf(csv_fp, "# Max frame delta us,%lld\n", delta_max_us);
        fprintf(csv_fp, "# Delta jitter us,%lld\n", delta_jitter);
        fprintf(csv_fp, "# Delta CV,%.6f\n", delta_cv);
        fprintf(csv_fp, "# Estimated FPS,%.3f\n", fps);
        fprintf(csv_fp, "# Deadline us,%lld\n", DEADLINE_US);
        fprintf(csv_fp, "# Deadline misses,%d\n", miss_count);
        fprintf(csv_fp, "# Miss percentage,%.2f\n", miss_percent);
    }
    
    fclose(csv_fp);
    cvDestroyWindow(window_name);
    
    printf("✓ Saved: %s\n", csv_name);
}

int main(int argc, char** argv)
{
    int dev = 0;
    
    if(argc > 1)
    {
        sscanf(argv[1], "%d", &dev);
        printf("Using camera device %d\n", dev);
    }
    else
    {
        printf("Using default camera device 0\n");
    }
    
    // Set SCHED_FIFO real-time scheduler
    struct sched_param param;
    param.sched_priority = 90;
    
    if(sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("ERROR: sched_setscheduler failed (need sudo/root)");
        printf("Continuing with default scheduler...\n");
    }
    else
    {
        printf("✓ Successfully set SCHED_FIFO scheduler (priority %d)\n", param.sched_priority);
    }
    
    // Open camera
    CvCapture* capture = (CvCapture *)cvCreateCameraCapture(dev);
    if(!capture)
    {
        printf("ERROR: could not open camera device %d\n", dev);
        return -1;
    }
    
    printf("\n======================================\n");
    printf("REAL-TIME TRANSFORM ANALYSIS - FIFO\n");
    printf("======================================\n");
    printf("Transforms: %d (Canny, Hough Lines, Hough Circles)\n", TRANSFORM_COUNT);
    printf("Resolutions: %d (160×120, 320×240, 640×480)\n", NUM_RESOLUTIONS);
    printf("Total Tests: %d\n", TRANSFORM_COUNT * NUM_RESOLUTIONS);
    printf("Frames per test: %d\n", MAX_FRAMES);
    printf("Deadlines: 70ms (default), 110ms (Lines@640), 130ms (Circles@640)\n");
    printf("Press 'q' during a test to skip to next\n");
    printf("======================================\n");
    
    // Run all combinations
    int test_num = 1;
    for(int t = 0; t < TRANSFORM_COUNT; t++)
    {
        for(int r = 0; r < NUM_RESOLUTIONS; r++)
        {
            printf("\n>>> Test %d/%d <<<\n", test_num++, TRANSFORM_COUNT * NUM_RESOLUTIONS);
            run_test(capture, (TransformType)t, resolutions[r]);
            
            // Small delay between tests
            usleep(1000000); // 1 second
        }
    }
    
    cvReleaseCapture(&capture);
    
    printf("\n======================================\n");
    printf("ALL TESTS COMPLETE!\n");
    printf("======================================\n");
    printf("Generated 9 CSV files:\n");
    for(int t = 0; t < TRANSFORM_COUNT; t++)
    {
        for(int r = 0; r < NUM_RESOLUTIONS; r++)
        {
            printf("  - fifo_%s_%s.csv\n", transform_names[t], resolutions[r].name);
        }
    }
    printf("\nRun analysis: python3 analyze_csv.py\n");
    
    return 0;
}
