#include <roborts_utils/base_param.h>

#include "roborts_utils/roborts_utils.h"
int main(int argc, char **argv) {
  LoadParam::InitParam("transponder", "config/transponder.jsonc");
  return 0;
}