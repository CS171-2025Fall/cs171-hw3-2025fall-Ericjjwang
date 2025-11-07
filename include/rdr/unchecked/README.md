# Unchecked SurfaceInteraction/Ray Variant

> In debugging, you might not use files in this directory since they are used for optimization purpose.

We do have a straight-forward example for you to demonstrate the necesscity, the result is

```txt
sizeof(StorageUnchecked)=64
sizeof(StorageChecked)=32
```

Notice that the unchecked version is twice compact as checked, this is a decision of compiler optimization, since the referenced variable can be relatively-indexed.
But anyway, if we insist on not writing the setter but to use const reference to expose most of the variables,
the size of the most-frequently-used struct will be doubled. Given that the checker is only for debugging purpose, we provide an extra unchecked version for efficiency. To switch between, you just need to change the template parameter.

## Example

```cpp
#include <format>
#include <iostream>
#include <type_traits>

struct alignas(32) StorageChecked {
  struct StorageCheckedInternal {
    int x;
    int y;
    int z;
  };

  const int &x{internal.x};
  const int &y{internal.y};
  const int &z{internal.z};
  StorageCheckedInternal internal;
};

struct alignas(32) StorageUnchecked {
  int x;
  int y;
  int z;
  StorageUnchecked &internal{*this};
};

template <bool EnableCheck = true>
struct Impl
    : public std::conditional_t<EnableCheck, StorageChecked, StorageUnchecked> {
  void set(int in_x, int in_y, int in_z) {
    this->internal.x = in_x;
    this->internal.y = in_x;
    this->internal.z = in_x;
  }
};

int main() {
  std::cout << std::format(
      "sizeof(StorageUnchecked)={}\nsizeof(StorageChecked)={}\n",
      sizeof(Impl<true>), sizeof(Impl<false>));
  // The implementation with check enabled, i.e., cannot directly write the
  // variable without extra setter function, in which you can perform checks,
  // for example, if the variable is correctly normalized.
  Impl<true> checked;
  Impl<false> unchecked;
  unchecked.x = 1;        // allowed
  unchecked.set(0, 1, 2); // allowed
  // checked.x = 1;       // not allowed
  checked.set(0, 1, 2);   // allowed
}
```
