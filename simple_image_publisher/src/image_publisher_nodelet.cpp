#include "simple_image_publisher/image_publisher.h"

#include <boost/thread.hpp>
#include <nodelet/nodelet.h>

namespace simple_image_publisher {

class ImagePublisherNodelet : public nodelet::Nodelet
{
public:
    ImagePublisherNodelet() : 
        thread_available_(false)
    {
    
    }

    ~ImagePublisherNodelet() {
        if(thread_available_) {
            thread_available_ = false;
            thread_->join();
        }
    }

private:
    virtual void onInit() {
        image_publisher_.reset(new ImagePublisher(getNodeHandle(), getPrivateNodeHandle()));
        image_publisher_->load();

        thread_available_ = true;
        thread_ = boost::shared_ptr<boost::thread>(
                new boost::thread(boost::bind(&ImagePublisherNodelet::spin, this)));
    }
    
    void spin() {
        while(thread_available_) {
            NODELET_INFO("publich camera image");
            image_publisher_->publish();
        }
    }

    bool thread_available_;
    boost::shared_ptr<ImagePublisher> image_publisher_;
    boost::shared_ptr<boost::thread> thread_;
};

} //namespace simple_image_publisher

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(simple_image_publisher::ImagePublisherNodelet, nodelet::Nodelet);
