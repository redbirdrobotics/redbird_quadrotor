#ifndef RR_SYNCHRONIZED_HPP_
#define RR_SYNCHRONIZED_HPP_

#include <functional>
#include <mutex>
#include <shared_mutex>
#include <type_traits>
#include <utility>


#undef DISALLOW_EVIL_CONSTRUCTORS
#define DISALLOW_EVIL_CONSTRUCTORS(TypeName) \
  TypeName(const TypeName&);                 \
  void operator=(const TypeName&)


namespace rr {


template <typename T>
class synchronized;


/**
 * @brief
 *    Make a synchronized<T> using template deduction.
 *
 * @sa synchronized
 */
template <typename T> auto
make_synchronized(T&& value) {
  return synchronized<T>{ std::forward<T>(value) };
}

/**
 * @brief
 *    Provides straightforward thread-synchronized access to template type.
 *
 * @note
 *    While access to the immediate object is synchronized, this class does not
 *  prevent non-synchronized access of pointer or reference members of the
 *  template type.
 */
template <typename T>
class synchronized final {
 private:
  using mutex_t = std::shared_timed_mutex;
  mutable mutex_t mutex_{};

 public:
  using value_t = std::remove_reference_t<T>;
  using read_lock = std::shared_lock<mutex_t>;
  using write_lock = std::unique_lock<mutex_t>;

 private:
  value_t value_;

 public:
  template <typename... Args>
  explicit synchronized(Args&&... args)
    : value_(std::forward<Args>(args)...)
  {}

  value_t get() const {
    read_lock l(mutex_);
    return value_;
  }

  template <typename U> void
  set(U&& new_value) {
    write_lock l(mutex_);
    value_ = std::forward<U>(new_value);
    std::function<void()> g;
  }

  template <typename Accessor>
  void use(Accessor&& access) const {
    read_lock l(mutex_);
    std::forward<Accessor>(access)(value_);
  }

  template <typename Mutator>
  void alter(Mutator&& func) {
    write_lock l(mutex_);
    std::forward<Mutator>(func)(value_);
  }

 private:
  DISALLOW_EVIL_CONSTRUCTORS(synchronized);
};

} // namespace rr


#endif // RR_SYNCHRONIZED_HPP_
