#ifndef _RANDOMNESS_PROVIDER_H_
#define _RANDOMNESS_PROVIDER_H_

#include <random>

class RandomnessProvider
{
 public:
  RandomnessProvider();

  double getRandomNumber();
  std::vector<unsigned int> getRandomPermutation(unsigned int size);

 private:
  std::mt19937 randomNumberGenerator;
};

#endif  // _RANDOMNESS_PROVIDER_H_