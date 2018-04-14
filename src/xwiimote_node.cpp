// C
#include <stdio.h>
#include <poll.h>
#include <xwiimote.h>
// C++
#include <string>
#include <ros/ros.h>

static struct xwii_iface *iface;

static bool run_iface(struct xwii_iface *iface)
{
  struct xwii_event event;
  int ret = 0, fds_num;
  struct pollfd fds[2];

  memset(fds, 0, sizeof(fds));
  fds[0].fd = 0;
  fds[0].events = POLLIN;
  fds[1].fd = xwii_iface_get_fd(iface);
  fds[1].events = POLLIN;
  fds_num = 2;

  ret = xwii_iface_watch(iface, true);
  if (ret) {
    ROS_FATAL("Error: Cannot initialize hotplug watch descriptor");
    return false;
  }

  while (true) {
    ret = poll(fds, fds_num, -1);
    if (ret < 0) {
      if (errno != EINTR) {
        ret = -errno;
        ROS_FATAL("Error: Cannot poll fds: %d", ret);
        break;
      }
    }

    ret = xwii_iface_dispatch(iface, &event, sizeof(event));
    if (ret) {
      if (ret != -EAGAIN) {
        ROS_FATAL("Error: Read failed with err:%d",
                  ret);
        break;
      }
    } else {
      switch (event.type) {
        case XWII_EVENT_GONE:
          ROS_INFO("Info: Device gone");
          fds[1].fd = -1;
          fds[1].events = 0;
          fds_num = 1;
          break;
        case XWII_EVENT_WATCH:
          ROS_INFO_THROTTLE(1, "XWII_EVENT_WATCH");
          break;
        case XWII_EVENT_KEY:
          ROS_INFO_THROTTLE(1, "XWII_EVENT_KEY");
          break;
        case XWII_EVENT_ACCEL:
          ROS_INFO_THROTTLE(1, "XWII_EVENT_ACCEL");
          break;
        case XWII_EVENT_IR:
          ROS_INFO_THROTTLE(1, "XWII_EVENT_IR");
          break;
        case XWII_EVENT_MOTION_PLUS:
          ROS_INFO_THROTTLE(1, "XWII_EVENT_MOTION_PLUS");
          break;
        case XWII_EVENT_NUNCHUK_KEY:
          ROS_INFO_THROTTLE(1, "XWII_EVENT_NUNCHUK_KEY");
          break;
        case XWII_EVENT_NUNCHUK_MOVE:
          ROS_INFO_THROTTLE(1, "XWII_EVENT_NUNCHUK_MOVE");
          break;
        case XWII_EVENT_CLASSIC_CONTROLLER_KEY:
          ROS_INFO_THROTTLE(1, "XWII_EVENT_CLASSIC_CONTROLLER_KEY");
          break;
        case XWII_EVENT_CLASSIC_CONTROLLER_MOVE:
          ROS_INFO_THROTTLE(1, "XWII_EVENT_CLASSIC_CONTROLLER_MOVE");
          break;
        case XWII_EVENT_BALANCE_BOARD:
          ROS_INFO_THROTTLE(1, "XWII_EVENT_BALANCE_BOARD");
          break;
        case XWII_EVENT_PRO_CONTROLLER_KEY:
          ROS_INFO_THROTTLE(1, "XWII_EVENT_PRO_CONTROLLER_KEY");
          break;
        case XWII_EVENT_PRO_CONTROLLER_MOVE:
          ROS_INFO_THROTTLE(1, "XWII_EVENT_WAXWII_EVENT_PRO_CONTROLLER_MOVETCH");
          break;
        case XWII_EVENT_GUITAR_KEY:
          ROS_INFO_THROTTLE(1, "XWII_EVENT_GUITAR_KEY");
          break;
        case XWII_EVENT_GUITAR_MOVE:
          ROS_INFO_THROTTLE(1, "XWII_EVENT_GUITAR_MOVE");
          break;
        case XWII_EVENT_DRUMS_KEY:
          ROS_INFO_THROTTLE(1, "XWII_EVENT_DRUMS_KEY");
          break;
        case XWII_EVENT_DRUMS_MOVE:
          ROS_INFO_THROTTLE(1, "XWII_EVENT_DRUMS_MOVE");
          break;
      }
    }
  }
  return ret;
}

////////////////////////////////////////////////////////////////////////////////

static char *get_dev(int num) {
  struct xwii_monitor *mon;
  char *ent;
  int i = 0;

  mon = xwii_monitor_new(false, false);
  if (!mon) {
    printf("Cannot create monitor\n");
    return NULL;
  }

  while ((ent = xwii_monitor_poll(mon))) {
    if (++i == num)
      break;
    free(ent);
  }

  xwii_monitor_unref(mon);

  if (!ent)
    printf("Cannot find device with number #%d\n", num);

  return ent;
}

////////////////////////////////////////////////////////////////////////////////

static void nunchuk_toggle(void) {
  int ret;

  if (xwii_iface_opened(iface) & XWII_IFACE_NUNCHUK) {
    xwii_iface_close(iface, XWII_IFACE_NUNCHUK);
    ROS_INFO("Info: Disable Nunchuk");
  } else {
    ret = xwii_iface_open(iface, XWII_IFACE_NUNCHUK);
    if (ret)
      ROS_FATAL("Error: Cannot enable Nunchuk: %d", ret);
    else
      ROS_INFO("Info: Enable Nunchuk");
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "xwiimote");
  ros::NodeHandle nh_private("~");
  if (argc < 2 || std::string(argv[1]) == "-h") {
    printf("Usage:\n");
    printf("  xwiishow [-h]: Show help\n");
    printf("  xwiishow list: List connected devices\n");
    printf("  xwiishow <num>: Show device with number #num\n");
    return -1;
  }
  std::string path;
  if (argv[1][0] != '/')
    path = get_dev(atoi(argv[1]));

  int ret = xwii_iface_new(&iface, path.size() ? path.c_str() : argv[1]);
  if (ret) {
    printf("Cannot create xwii_iface '%s' err:%d\n",
           argv[1], ret);
    return -1;
  }

  ret = xwii_iface_open(iface,
                        xwii_iface_available(iface) |
                        XWII_IFACE_WRITABLE);
  if (ret) {
    ROS_FATAL("Error: Cannot open interface: %d", ret);
    return -1;
  }

  nunchuk_toggle();
  nunchuk_toggle();
  ret = run_iface(iface);
  if (ret) {
    ROS_FATAL("Program failed; press any key to exit");
    return -1;
  }
  xwii_iface_unref(iface);
}
