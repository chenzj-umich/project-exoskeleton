import numpy as np

class Mat:
  def add(A, B):
      return np.add(A, B)

  def subtract(A, B):
      return np.subtract(A, B)

  def multiply(A, B):
      return np.dot(A, B)

  def inv(A):
      if np.linalg.det(A) == 0:
          raise ValueError("Matrix is singular and cannot be inverted.")
      return np.linalg.inv(A)

  def det(A):
      return np.linalg.det(A)

# def rotation
