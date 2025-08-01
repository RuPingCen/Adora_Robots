extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}
#include <string>
#include <assert.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <rerun.hpp>
#include <thread>
#include <pthread.h>
#include <cmath>
#include <time.h>
#include <sys/time.h>
#include <iomanip>

using namespace std;


bool read_map(const string globalmap_pcd, rerun::RecordingStream& rec)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile(globalmap_pcd, *cloud);

    std::vector<rerun::Position3D> map_points;
    std::vector<rerun::Color> map_colors;  // 颜色根据强度映射

    for (const auto& pt : cloud->points) {
        map_points.emplace_back(pt.x, pt.y, pt.z);
        map_colors.emplace_back(255, 0, 0);
    }

    rec.log("map_points_static", rerun::Points3D(map_points).with_colors(map_colors).with_radii({0.05f}));
    std::cout << "Loaded " << cloud->points.size()<< " map to Rerun." << std::endl;

    return true;
}


bool points_to_rerun(const char *bytes, int32_t size, rerun::RecordingStream& rec)
{
    if (size <= 0)
    {
        std::cerr << "Error: Point cloud size <= 0!" << std::endl;
        return false;
    }
    // struct timeval tv;
    // gettimeofday(&tv, NULL);//获取时间
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    std::vector<rerun::Position3D> points;
    std::vector<rerun::Color> colors;


    cloud->header.seq = *(std::uint32_t *)bytes;
    cloud->header.stamp = *(std::uint64_t *)(bytes + 8);
    cloud->header.frame_id = "rslidar";
    cloud->width = size;
    cloud->height = 1;
    cloud->is_dense = true;

    for (int i = 0; i < size; i++)
    {
        pcl::PointXYZI tem_point;
        tem_point.x = *(float *)(bytes + 16 + 16 * i);
        tem_point.y = *(float *)(bytes + 16 + 4 + 16 * i);
        tem_point.z = *(float *)(bytes + 16 + 8 + 16 * i);
        tem_point.intensity = *(float *)(bytes + 16 + 12 + 16 * i);
        points.emplace_back(tem_point.x, tem_point.y, tem_point.z);
        colors.emplace_back(0, 255, 0);
    }
    rec.log("live_points", rerun::Points3D(points).with_colors(colors).with_radii({0.05f}));
    std::cout << "Logging " << points.size() << " points to Rerun." << std::endl;
    // struct timeval tv_1;
    // gettimeofday(&tv_1, NULL);//获取时间
    // auto start = tv.tv_sec + tv.tv_usec * 1e-6;
    // auto end = tv_1.tv_sec + tv_1.tv_usec * 1e-6;
    // auto all_time = end - start;
    // std::cout << "Time: " << all_time << std::endl;
    // double stamp = (double)cloud->header.stamp*1.0*1e-6;

    return true;
}

int run(void *dora_context, rerun::RecordingStream& rec)
{
    while (true)
    {
        void * event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);
            int32_t point_len = (data_len - 16) / 16;          
            if (strncmp("pointcloud", data_id, 10) == 0)
            {
                auto res = points_to_rerun(data, point_len, rec);
                if (!res)
                {
                    std::cerr << "Error: Failed to send point cloud to Rerun!" << std::endl;
                }
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
        }                // auto cloud = bytes_to_cloud(data, point_len);
                // if (!cloud || cloud->empty())
                // {
                //     std::cerr << "Error: Failed to convert bytes to point cloud!" << std::endl;
                //     return -1;
                // }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }
    return 0;
}

int main()
{
    std::cout << "rerun_test" << std::endl;
    void* dora_context = init_dora_context_from_env();

    rerun::RecordingStream rec("lidarpoints_viewer");
    rec.spawn().exit_on_failure();

    std::string globalmap_pcd ="./map.pcd";
    auto map_load = read_map(globalmap_pcd, rec);
    if (!map_load)
    {
        std::cerr << "Error: Failed to open map!" << std::endl;
    }

    auto ret = run(dora_context, rec);
    free_dora_context(dora_context);
    std::cout << "END rerun_test" << std::endl;
    return ret;
}