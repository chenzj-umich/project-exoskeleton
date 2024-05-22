# from ulab import numpy as np
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


# def overriding operator for list
def list_div(self, divisor):
    divided_numbers = [x / divisor for x in self]
    return divided_numbers

def list_add(self, other):
    if len(self) != len(other):
        raise ValueError("Both lists must have the same length for element-wise addition.")
    return [a + b for a, b in zip(self, other)]

def list_sub(self, other):
    if len(self) != len(other):
        raise ValueError("Both lists must have the same length for element-wise addition.")
    return [a - b for a, b in zip(self, other)]
