#include "jvs_host.h"

int main(void) {
  JVS_HOST_Init();

  for (;;) {
    JVS_HOST_Run();
  }
}