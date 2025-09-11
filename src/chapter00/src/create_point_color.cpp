#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cmath>

typedef pcl::PointXYZRGBA pointcolor;
typedef pcl::PointXYZ pointxyz;

// 金字塔参数配置
struct PyramidConfig {
    float base_size = 5.0;      // 底面边长
    float height = 5.0;         // 总高度
    int points_per_edge = 100;   // 每条边的采样点数
    float layer_height = 0.1;   // 层间距（用于实心金字塔）
    uint8_t r = 120;
    uint8_t g = 120;
    uint8_t b = 0;
    uint8_t a = 120;
};

// 创建金字塔表面点云
void createPyramidSurface(pcl::PointCloud<pointcolor>& cloud, const PyramidConfig& config) {
    // 生成底面四条边的点
    float half_base = config.base_size / 2.0;
    for(int i = 0; i < config.points_per_edge; i++) {
        float t = static_cast<float>(i) / (config.points_per_edge - 1);
        
        // 底面四个边
        cloud.push_back(pointcolor(-half_base + config.base_size * t, -half_base, 0, 255, 0, 0, 0));  // 前边
        cloud.push_back(pointcolor(half_base, -half_base + config.base_size * t, 0, 0, 255, 0, 0));   // 右边
        cloud.push_back(pointcolor(half_base - config.base_size * t, half_base, 0, 0, 0, 255, 0));    // 后边
        cloud.push_back(pointcolor(-half_base, half_base - config.base_size * t, 0, 0, 0, 0, 255));   // 左边
    }
    
    // 生成四条棱线的点
    for(int i = 0; i < config.points_per_edge; i++) {
        float t = static_cast<float>(i) / (config.points_per_edge - 1);
        float x = half_base * (1 - t);
        float y = half_base * (1 - t);
        float z = config.height * t;
        
        // 四个角点连线
        cloud.push_back(pointcolor(-x, -x, z, 255, 0, 0, 0));  // 前左棱
        cloud.push_back(pointcolor(x, -x, z, 255, 0, 0, 0));   // 前右棱
        cloud.push_back(pointcolor(x, x, z, 255, 0, 0, 0));    // 后右棱
        cloud.push_back(pointcolor(-x, x, z, 255, 0, 0, 0));   // 后左棱
    }
    
    // 生成顶点
    cloud.push_back(pointcolor(0, 0, config.height, 255, 0, 0, 0));
}

// 创建实心金字塔（层叠结构）
void createSolidPyramid(pcl::PointCloud<pointcolor>& cloud, const PyramidConfig& config) {
    float half_base = config.base_size / 2.0;
    int layers = static_cast<int>(config.height / config.layer_height);
    
    for(int layer = 0; layer < layers; layer++) {
        float z = layer * config.layer_height;
        float current_base = config.base_size * (1 - z / config.height);  // 每层逐渐缩小
        float half_current = current_base / 2.0;
        
        // 本层的点密度随高度递减
        int points_this_layer = std::max(4, config.points_per_edge * (layers - layer) / layers);
        
        for(int i = 0; i < points_this_layer; i++) {
            float t = static_cast<float>(i) / (points_this_layer - 1);
            
            // 绘制本层的四条边
            cloud.push_back(pointcolor(-half_current + current_base * t, -half_current, z, config.r, config.g, config.b, config.a));
            cloud.push_back(pointcolor(half_current, -half_current + current_base * t, z, config.r, config.g, config.b, config.a));
            cloud.push_back(pointcolor(half_current - current_base * t, half_current, z, config.r, config.g, config.b, config.a));
            cloud.push_back(pointcolor(-half_current, half_current - current_base * t, z, config.r, config.g, config.b, config.a));
        }
    }
}

int main() {
    pcl::PointCloud<pointcolor> cloud;
    PyramidConfig config;
    
    // 选择生成模式：表面框架 or 实心结构
    // createPyramidSurface(cloud, config);  // 表面框架
    createSolidPyramid(cloud, config);      // 实心结构

    // 设置点云元数据
    cloud.width = cloud.size();
    cloud.height = 1;
    cloud.is_dense = true;

    // 保存点云文件
    std::string file_path = "./src/chapter01/pcd/pyramid_color.pcd";
    if (pcl::io::savePCDFileASCII(file_path, cloud) == 0) {
        std::cout << "成功生成金字塔点云: " << file_path << " (" << cloud.size() << " 个点)\n";
    } else {
        std::cerr << "保存失败! 错误原因:\n"
                  << "1. 目录不存在: " << file_path << "\n"
                  << "2. 写入权限不足\n"
                  << "3. 路径无效\n";
    }

    return 0;
}