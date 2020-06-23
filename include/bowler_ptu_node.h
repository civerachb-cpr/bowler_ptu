#ifndef __BOWLER_PTU_HEADER__
#deinfe __BOWLER_PTU_HEADER__

#include <string>

namespace bowler_ptu
{
  class BowlerPtuNode{
  public:
    BowlerPtuNode(std::string port, int baud);
    ~BowlerPtuNode();

  private:
    std::string _port;
    int _baud;
  }
}

#endif
