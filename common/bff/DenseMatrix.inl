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

#include "DenseMatrix.h"
#include "SparseMatrix.h"

namespace bff {

inline DenseMatrix::DenseMatrix(size_t m, size_t n) : mat_(m, n)
{
    mat_.setZero();
}

inline DenseMatrix DenseMatrix::identity(size_t m, size_t n)
{
    EigenMatrix mat = EigenMatrix::Identity(m, n);
    return mat;
}

inline DenseMatrix DenseMatrix::ones(size_t m, size_t n)
{
    EigenMatrix mat = EigenMatrix::Ones(m, n);
    return mat;
}

inline DenseMatrix DenseMatrix::transpose() const
{
    return EigenMatrix(mat_.transpose());
}

inline DenseMatrix DenseMatrix::submatrix(size_t r0, size_t r1, size_t c0, size_t c1) const
{
	size_t m = r1 - r0;
	size_t n = c1 - c0;
	DenseMatrix A(m, n);
    A.mat() = mat().block(r0, c0, m, n);
	return A;
}

inline DenseMatrix DenseMatrix::submatrix(const std::vector<int>& r,
										  const std::vector<int>& c) const
{
	size_t m = r.size();
	size_t n = c.size();
	DenseMatrix A(m, n);

	for (size_t i = 0; i < m; i++) {
		for (size_t j = 0; j < n; j++) {
			A(i, j) = (*this)(r[i], c[j]);
		}
	}

	return A;
}


inline DenseMatrix operator*(const SparseMatrix& A, const DenseMatrix& X)
{
    return DenseMatrix::EigenMatrix(A.mat() * X.mat());
}

inline DenseMatrix hcat(const DenseMatrix& A, const DenseMatrix& B)
{
	size_t m = A.nRows();
	size_t n1 = A.nCols();
	size_t n2 = B.nCols();
	DenseMatrix C(m, n1 + n2);

	for (size_t i = 0; i < m; i++) {
		for (size_t j = 0; j < n1; j++) {
			C(i, j) = A(i, j);
		}

		for (size_t j = 0; j < n2; j++) {
			C(i, n1 + j) = B(i, j);
		}
	}

	return C;
}

inline DenseMatrix vcat(const DenseMatrix& A, const DenseMatrix& B)
{
	size_t m1 = A.nRows();
	size_t m2 = B.nRows();
	size_t n = A.nCols();
	DenseMatrix C(m1 + m2, n);

	for (size_t j = 0; j < n; j++) {
		for (size_t i = 0; i < m1; i++) {
			C(i, j) = A(i, j);
		}

		for (size_t i = 0; i < m2; i++) {
			C(m1 + i, j) = B(i, j);
		}
	}

	return C;
}

} // namespace bff
