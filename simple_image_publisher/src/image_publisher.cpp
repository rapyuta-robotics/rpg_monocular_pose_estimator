#include <simple_image_publisher/image_publisher.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh, nh_private("~");
    simple_image_publisher::ImagePublisher image_publisher(nh, nh_private);
    image_publisher.load();

    while(nh.ok()) {
        ros::spinOnce();
        image_publisher.publish();
    }

    return 0;
}
