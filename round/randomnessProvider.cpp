#include "randomnessProvider.h"

#include <algorithm>

RandomnessProvider::RandomnessProvider()
{
    std::random_device rd;
    randomNumberGenerator = std::mt19937(rd());
}

int RandomnessProvider::getRandomNumber(unsigned int limit)
{
    std::uniform_int_distribution<> uniformDistribution(0, limit - 1);

    return uniformDistribution(randomNumberGenerator);
}

std::vector<int> RandomnessProvider::getRandomPermutation(unsigned int size)
{
    std::vector<int> indexes(size);
    for(int i = 0; i < indexes.size(); i++)
    {
        indexes[i]=i;
    }
 
    std::shuffle(indexes.begin(), indexes.end(), randomNumberGenerator);

    return std::move(indexes);
}
