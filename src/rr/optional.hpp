#ifndef RR_OPTIONAL_HPP_
#define RR_OPTIONAL_HPP_

#include <utility>

namespace rr {

/**
 * @brief
 *    A value communicated to be optionally 'set' or 'not set', regardless of
 *  the underlying value. This may be useful for sending logical groups of
 *  setpoint values, where some should be ignored.
 */
template <typename T>
class optional final {
 public:
  using value_t = T;

  /**
   * @brief
   *    Get the value, whether 'set' or not.
   */
  value_t get() const { return v_; }

  /**
   * @brief
   *    Set the value.
   *
   * @note
   *    This implicitly 'set's this optional.
   */
  template <typename U>
  void set(U&& new_value) { is_set_ = true; v_ = std::forward<U>(new_value); }

  /**
   * @brief
   *    Indicate the value is 'set'.
   */
  void set() { is_set_ = true; }

  /**
   * @brief
   *    Without changing the underlying value, indicate this optional as not
   *  'set'.
   */
  void unset() { is_set_ = false; }

  /**
   * @brief
   *    Test whether this value
   */
  bool is_set() const { return is_set_; }

  template <typename U>
  const value_t& operator=(U&& new_value) {
    set(std::forward<U>(new_value));
    return v_;
  }

 private:
  value_t v_{};
  bool is_set_ = false;
};

} // namespace rr

#endif // RR_OPTIONAL_HPP_
