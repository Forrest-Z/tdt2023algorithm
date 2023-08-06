#ifndef __TRANSPONDER_H
#define __TRANSPONDER_H

#include "roborts_utils/base_communicator.h"
namespace tdt_transponder {
class Transponder {
 private:
  tdttoolkit::UDPServer udp_server_;
  tdttoolkit::UDPClient udp_client_;
  ;

 public:
  Transponder();
  ;
};
}  // namespace tdt_transponder
#endif