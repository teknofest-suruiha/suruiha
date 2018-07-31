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

#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include <vector>
#include <string>
#include <sstream>
#include "string.h"
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <suruiha_gazebo_plugins/score_calculator/dtw.h>
#include <gazebo/common/Console.hh>

using namespace DTW;

SimpleDTW::SimpleDTW()
{
    initialized_ = false;
}

SimpleDTW::SimpleDTW(size_t x_size, size_t y_size)
{
    SimpleDTW::Initialize(x_size, y_size);
}

void SimpleDTW::Initialize(size_t x_size, size_t y_size)
{
    x_dim_ = x_size + 1;
    y_dim_ = y_size + 1;
    // Resize the data
    data_.resize(x_dim_ * y_dim_, 0.0);
    //Populate matrix with starting values
    SetInDTWMatrix(0, 0, 0.0);
    for (size_t i = 1; i < x_dim_; i++)
    {
        SetInDTWMatrix(i, 0, INFINITY);
    }
    for (size_t i = 1; i < y_dim_; i++)
    {
        SetInDTWMatrix(0, i, INFINITY);
    }
    initialized_ = true;
}

double SimpleDTW::EvaluateWarpingCost(std::vector< std::vector<double> >& sequence_1, std::vector< std::vector<double> >& sequence_2)
{
    // Sanity checks
    if (sequence_1.size() == 0 || sequence_2.size() == 0)
    {
        return INFINITY;
    }
    if (sequence_1[0].size() != sequence_2[0].size())
    {
        gzdbg << "sequence_1[0].size():" << sequence_1[0].size() << " sequence_2[0].size():" << sequence_2[0].size() << std::endl;
//        DumpAll(sequence_2);
        throw std::invalid_argument("Sequences for evaluation have different element sizes");
    }
    // Safety checks
    if (!initialized_ || sequence_1.size() >= x_dim_ || sequence_2.size() >= y_dim_)
    {
        gzdbg << "Automatically resizing DTW matrix to fit arguments seq1.size:" << sequence_1.size() << " seq2.size:" << sequence_2.size() << std::endl;
//        gzdbg << "x_dim:" << x_dim_ << " y_dim:" << y_dim_ << std::endl;
        SimpleDTW::Initialize(sequence_1.size(), sequence_2.size());
    }
    //Compute DTW cost for the two sequences
    for (unsigned int i = 1; i <= sequence_1.size(); i++)
    {
        for (unsigned int j = 1; j <= sequence_2.size(); j++)
        {
            double index_cost = distance(sequence_1[i - 1], sequence_2[j - 1]);
            double prev_cost = 0.0;
            // Get the three neighboring values from the matrix to use for the update
            double im1j = GetFromDTWMatrix(i - 1, j);
            double im1jm1 = GetFromDTWMatrix(i - 1, j - 1);
            double ijm1 = GetFromDTWMatrix(i, j - 1);
            // Start the update step
            if (im1j < im1jm1 && im1j < ijm1)
            {
                prev_cost = im1j;
            }
            else if (ijm1 < im1j && ijm1 < im1jm1)
            {
                prev_cost = ijm1;
            }
            else
            {
                prev_cost = im1jm1;
            }
            // Update the value in the matrix
            SetInDTWMatrix(i, j, index_cost + prev_cost);
        }
    }
    //Return total path cost
    return GetFromDTWMatrix(sequence_1.size(), sequence_2.size());
}


//void SimpleDTW::DumpAll(std::vector<std::vector<double> > seq) {
//    std::cout << "START PRINTING SEQUENCE DATA" << std::endl;
//    for (unsigned int i = 0; i < seq.size(); i++) {
//        std::cout << "seq.at(" << i << ").size:" << seq.at(i).size() << std::endl;
//        for (unsigned int j = 0; j < seq.at(i).size(); j++) {
//            std::cout << seq.at(i).at(j) << "-";
//        }
//        std::cout << std::endl;
//    }
//    std::cout << "END PRINTING SEQUENCE DATA" << std::endl;
//}