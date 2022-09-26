#include "randomnessProvider.h"

RandomnessProvider::RandomnessProvider()
{
  std::random_device rd;
  randomNumberGenerator = std::mt19937(rd());
}

double RandomnessProvider::getRandomNumber()
{
  std::uniform_real_distribution<double> uniformDistribution(0.0, 1.0);

  return uniformDistribution(randomNumberGenerator);
}

std::vector<unsigned int> RandomnessProvider::getRandomPermutation(unsigned int size)
{
  std::vector<unsigned int> indexes(size);
  for (auto i = 0U; i < indexes.size(); i++)
  {
    indexes[i] = i;
  }

  std::shuffle(indexes.begin(), indexes.end(), randomNumberGenerator);

  return std::move(indexes);
}
