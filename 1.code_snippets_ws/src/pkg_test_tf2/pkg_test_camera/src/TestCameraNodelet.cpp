//#include "pkg_test_camera/include/pkg_test_camera/TestCamera.h"
#include "pkg_test_camera/TestCamera.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace pkg_test_camera{
class TestCameraNodelet : public nodelet::Nodelet
{
public:
  TestCameraNodelet()
  {

  }

  ~TestCameraNodelet()
  {

  }

  virtual void onInit()
  {
    p_.reset(new TestCamera(getNodeHandle(), getPrivateNodeHandle()));
  }

  boost::shared_ptr<pkg_test_camera::TestCamera> p_;
};
PLUGINLIB_EXPORT_CLASS(pkg_test_camera::TestCameraNodelet, nodelet::Nodelet);
}
