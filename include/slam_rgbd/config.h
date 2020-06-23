//
// Declare the Config class
//

#ifndef SLAMRGBD_CONFIG_H
#define SLAMRGBD_CONFIG_H
#include "slam_rgbd/common_include.h"

namespace slamrgbd {
    class Config {
    private:
        cv::FileStorage file_;
        static Config * config_;

        Config()=default;
        Config(const Config & obj)=delete;
        Config & operator=(const Config & obj)=delete;

    public:
        ~Config();

        static void SetParameterFile(const string & filename);
        template<typename T>
        static T get(const string & key) {return T(Config::config_->file_[key]); }
    };

}


#endif //SLAMRGBD_CONFIG_H
