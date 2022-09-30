/*  SPDX-License-Identifier: MIT

    The MIT License (MIT)

    Copyright (c) 2017 Rohan Sawhney and Keenan Crane

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal in
    the Software without restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
    Software, and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
    INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
    PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
    OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "SparseMatrix.h"
#include <cstring>

namespace bff {

inline SparseMatrix::SparseMatrix(size_t m, size_t n):
    mat_(m, n)
{
}


inline SparseMatrix SparseMatrix::identity(size_t m, size_t n)
{
    SparseMatrix mat(m, n);
    mat.mat().setIdentity();
    return mat;
}


inline SparseMatrix SparseMatrix::diag(const DenseMatrix& d)
{
    EigenMatrix A(d.nRows(), d.nRows());
    for (std::size_t i = 0; i < d.nRows(); i++){
        A.insert(i, i) = d(i);
    }
    return A;
}

inline SparseMatrix SparseMatrix::transpose() const
{
    return EigenMatrix(mat_.transpose());
}

inline SparseMatrix SparseMatrix::submatrix(size_t r0, size_t r1, size_t c0, size_t c1) const
{
    size_t m = r1 - r0;
    size_t n = c1 - c0;
    SparseMatrix A(m, n);
    A.mat() = mat().block(r0, c0, m, n);
    return A;
}


inline SparseMatrix SparseMatrix::submatrix(const std::vector<int>& r,
											const std::vector<int>& c) const
{
    EigenMatrix A(r.size(), c.size());

    for (int ci = 0; ci < c.size(); ++ci) {
        int ri = 0;

        for (Eigen::SparseMatrix<double>::InnerIterator it(mat(), c[ci]); it; ++it)
        {
            while (r[ri] < it.row() && ri < r.size()) {
                ri++;
            }
            if (ri == r.size()) {
                break;
            }
            if (r[ri] == it.row()) {
                A.insert(ri, ci) = it.value();
            }
        }
    }
	return A;
}

} // namespace bff
