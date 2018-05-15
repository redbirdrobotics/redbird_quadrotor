#include <gmock/gmock.h>

#include <rr/iarc/m7a/m7a.hpp>

#include <memory>
#include <utility>
#include <functional>
#include <vector>

using namespace ::testing;
using namespace rr;
using namespace rr::iarc;
using namespace rr::iarc::m7;


/*
 *  Imagine this coordinate system:
 *
 *  { position (x,y,z) | x,y,z given in m }
 *  { velocity <x,y,z> | x,y,z given in m/s }
 *
 *        +y ^
 *           |
 *           |
 *                    GREEN
 *        (0,20)___________________(20,20)
 *            |                    |
 *            |                    |
 *            |                    |
 *            |                    |
 *            |                    |
 *            |                    |
 *            | _. v = <2,3,0>     |
 * drone      | /|                 |
 * origin --> |/___________________|
 *        (0,0)        RED       (20,0)  --> +x
 *          /
 *         /
 *       \/_ +z (away from Earth)
 *     (0,0,3)
 *
 */

struct vec2 { double x; double y; };
struct vec3 { double x; double y; double z; };

class roomba {
 public:
  virtual vec2
  position() const = 0;

  virtual vec2
  velocity() const = 0;

  virtual bool
  is_obstacle() const = 0;

  virtual ~roomba() {}
};

using const_roombalist_ptr = std::shared_ptr<const std::vector<
  const std::unique_ptr<const roomba>>>;

class m7a_player {
 public:
  virtual vec3
  desired_position() = 0;

  virtual void
  set_roombas(const_roombalist_ptr roombas) = 0;

  virtual ~m7a_player() {}
};

class m7a_player_base {
 protected:
  const_roombalist_ptr roombas_;

 public:
  virtual void
  set_roombas(const_roombalist_ptr roombas) {
    roombas_ = roombas;
  }

  virtual ~m7a_player_base() {}
};

class point_scorer : public m7a_player_base {
 public:
  virtual vec3
  desired_position() {

  }

 virtual void
 set_roombas(const_roombalist_ptr roombas) = 0;
};
class point_protector : public m7a_player {};
class obstacle_avoider : public m7a_player {};

class MockM7aPlayer {
 public:
  MOCK_METHOD1(set_roombas, void(const_roombalist_ptr roombas));
};

class MockRoomba {
 public:
  MOCK_CONST_METHOD0(position, vec2());
  MOCK_CONST_METHOD0(velocity, vec2());
  MOCK_CONST_METHOD0(is_obstacle, bool());
};


TEST(Iarc, m7a) {
  MockM7aPlayer p{};
  auto roombas = std::make_shared<std::vector<
      const std::unique_ptr<const roomba>>>();

  roombas->emplace_back();
}
