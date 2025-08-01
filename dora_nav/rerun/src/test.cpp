#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <rerun.hpp>
#include <vector>

int main() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointXYZI point1,point2,point3;
    point1.x = 1.0; point1.y = 2.0; point1.z = 3.0; point1.intensity = 1.5;
    point2.x = 2.0; point2.y = 3.0; point2.z = 5.0; point2.intensity = 2.8;
    point3.x = 3.0; point3.y = 5.0; point3.z = 7.0; point3.intensity = 1.0;
    cloud->push_back(point1);
    cloud->push_back(point2);
    cloud->push_back(point3);
    // cloud->push_back(4.0f, 6.0f, 8.0f, 0.3f);

    std::vector<rerun::Position3D> points;
    std::vector<rerun::Color> colors;  // 颜色根据强度映射

    for (const auto& pt : cloud->points) {
        points.emplace_back(pt.x, pt.y, pt.z);

        uint8_t intensity_color = static_cast<uint8_t>(pt.intensity * 100);
        colors.emplace_back(intensity_color, intensity_color, intensity_color);
    }

    rerun::RecordingStream rec("pcl_to_rerun");  // 录制流
    rec.spawn().exit_on_failure();  // 启动 Rerun Viewer

    rec.log("point_cloud", rerun::Points3D(points).with_colors(colors).with_radii(0.05));

    return 0;
}