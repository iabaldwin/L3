#include "Dataset.h"

namespace L3
{
  Dataset::Dataset(const std::string& target)  : start_time(0) {
    std::string tmp(target);

    if (*(tmp.end()-1) != '/') {
      tmp.insert(tmp.end(), '/');
    }

    // From the root path
    root_path /= tmp;

    root_path.remove_filename();

    // Dataset name
    dataset_name = root_path.leaf().string();

    // Append the location of the binary data
    root_path /= "L3";

    // Does the directory exist?
    if (!boost::filesystem::is_directory(root_path)) {
      LOG(ERROR) << root_path << " does not exist ...";
      throw L3::no_such_folder();
    }

    lookup[".ins"]   = INS_file;
    lookup[".lhlv"]  = LHLV_file;
    lookup[".lidar"] = LIDAR_file;
    lookup[".sm"]    = SM_file;
  }

  Dataset::~Dataset() {
    if (pose_reader) {
      pose_reader->stop();
    }

    if (LHLV_reader) {
      LHLV_reader->stop();
    }

    for (std::map< std::string, boost::shared_ptr< SlidingWindow<L3::LMS151> > >::iterator it = LIDAR_readers.begin(); it != LIDAR_readers.end(); it++) {
      it->second->stop();
    }

    for(std::list< boost::shared_ptr< Poco::Thread > >::iterator it= threads.begin(); it != threads.end(); it++) {
      (*it)->join();
    }
  }

  std::ostream& operator<<(std::ostream& o, const Dataset& dataset) {
    std::copy(boost::filesystem::directory_iterator(dataset.root_path),
        boost::filesystem::directory_iterator(),
        std::ostream_iterator<boost::filesystem::directory_entry>(o, "\n"));

    o << "Pose Reader:"     << dataset.pose_reader << std::endl;
    o << "LHLV Reader:"     << dataset.LHLV_reader << std::endl;
    o << "Velocity Reader:" << dataset.velocity_reader << std::endl;
    return o;
  }

  bool Dataset::validate() {
    /*
     *For any dataset, there should be:
     *  1.  1 x INS file
     *  2.  1 X LHLV file
     *  3.  N x LIDAR files
     *
     */
    boost::filesystem::directory_iterator it(root_path);

    while(it != boost::filesystem::directory_iterator()) {

      switch (lookup[boost::filesystem::extension(*it)]) {
        case INS_file:
          OxTS_ins = *it;
          break;

        case LIDAR_file:
          LIDARs.push_front(*it);
          break;

        case LHLV_file:
          OxTS_lhlv = *it;
          break;

        case SM_file:
          SM_vel= *it;
          break;

        default:
          LOG(WARNING) << "Unknown type, " << *it;
          break;
      }
      it++;
    }

    // Validate logic
    if (!(boost::filesystem::exists(OxTS_ins)) ||
        !(boost::filesystem::exists(OxTS_lhlv)) ||
        !(boost::filesystem::exists(SM_vel)) ||
        (LIDARs.size()) == 0) {
      return false;
    } else {
      return true;
    }
  }

  bool Dataset::load() {
    // Pose Reader
#ifndef NDEBUG
    LOG(INFO) <<  "OXTS" <<  OxTS_ins.path().string();
#endif
    pose_reader = L3::WindowerFactory<L3::SE3>::constantTimeWindow(OxTS_ins.path().string(), 30) ;
    CHECK(pose_reader->initialise()) << "Failed to initialise : [" << OxTS_ins.path() << "]";
    runnables.push_back(pose_reader);

    // LHLV Reader
#ifndef NDEBUG
    LOG(INFO) <<  "LHLV" << OxTS_lhlv.path().string();
#endif
    LHLV_reader = L3::WindowerFactory<L3::LHLV>::constantTimeWindow(OxTS_lhlv.path().string(), 30) ;
    CHECK(LHLV_reader->initialise());
    runnables.push_back(LHLV_reader);

    //Velocity reader
#ifndef NDEBUG
    LOG(INFO) << "Velocity" << SM_vel.path().string();
#endif
    velocity_reader = L3::WindowerFactory<L3::SMVelocity>::constantTimeWindow(SM_vel.path().string(), 30) ;
    CHECK(velocity_reader->initialise());
    runnables.push_back(velocity_reader);

    // Load LIDARs
    std::list< boost::filesystem::directory_entry >::iterator it = LIDARs.begin();

    while (it != LIDARs.end()) {
      boost::shared_ptr< SlidingWindow<L3::LMS151> > reader = L3::WindowerFactory<L3::LMS151>::constantTimeWindow((*it).path().string(), 30);
      CHECK_NOTNULL(reader);
      std::string LIDAR_name = (*it).path().stem().string();
      CHECK(reader->initialise()) << "Failed to initialize [" << LIDAR_name << "]";
      LOG(INFO) << "Inserting " << LIDAR_name;
      LIDAR_readers.insert(std::make_pair(LIDAR_name, reader));
      runnables.push_back(reader);
      it++;
    }

    for(std::list< boost::shared_ptr<Poco::Runnable> >::iterator it = runnables.begin(); it != runnables.end(); it++) {
      Poco::Thread* thread = new Poco::Thread();
      CHECK_NOTNULL(thread);
      thread->start(*(*it));
      threads.push_back(boost::shared_ptr< Poco::Thread >(thread));
    }

    // Get the dataset time - by definition, this is the time of the first pose
    start_time = pose_reader->window.begin()->first;

    return true;
  }

} // L3
