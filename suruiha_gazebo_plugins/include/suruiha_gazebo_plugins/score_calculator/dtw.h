/*
Copyright (c) 2014, Calder Phillips-Grafflin (calder.pg@gmail.com)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdlib.h>
#include <stdio.h>
#include <vector>

#ifndef DTW_H
#define DTW_H

namespace DTW
{

class SimpleDTW
{
private:

    std::vector<double> data_;
    size_t x_dim_;
    size_t y_dim_;
    bool initialized_;

    void Initialize(size_t x_size, size_t y_size);

    inline size_t GetDataIndex(size_t x, size_t y)
    {
        return (x * y_dim_) + y;
    }

    inline double GetFromDTWMatrix(size_t x, size_t y)
    {
        return data_[GetDataIndex(x, y)];
    }

    inline void SetInDTWMatrix(size_t x, size_t y, double val)
    {
        data_[GetDataIndex(x, y)] = val;
    }

    inline double distance(const std::vector<double>& p1, const std::vector<double>& p2) {
        double total = 0.0;
        for (unsigned int i = 0; i < p1.size(); i++)
        {
            total += pow((p1[i] - p2[i]), 2);
        }
        return sqrt(total);
    }

public:

    SimpleDTW(size_t x_dim, size_t y_dim);

    SimpleDTW();

    ~SimpleDTW() {}

    double EvaluateWarpingCost(std::vector<std::vector<double> >& sequence_1, std::vector<std::vector<double> >& sequence_2);

//    void DumpAll(std::vector<std::vector<double> > seq);

};

}

#endif // DTW_H
