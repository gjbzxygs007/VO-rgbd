//
// The definition of Config class
//

#include "slam_rgbd/config.h"

namespace slamrgbd {
    void Config::SetParameterFile(const string & filename) {
        if (config_ == NULL) {
            config_ = shared_ptr<Config>(new Config());
        }
        config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
        if (config_->file_.isOpened() == false) {
            cerr << "Parameter file " << filename << " does not exist." << endl;
            config_->file_.release();
            return;
        }
    }

    Config::~Config() {
        if (file_.isOpened()) {
            file_.release();
        }
    }

    shared_ptr<Config> Config::config_ = NULL;
}

