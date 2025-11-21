import numpy as np
from sklearn.linear_model import LinearRegression

nose = np.array([0,7,154])
lefteye = np.array([-2,5,156])
righteye = np.array([2, 5, 156])
leftear = np.array([-6,0,155])
rightear = np.array([6,0,155])

# take two ears and a nose/eye
leftear2ear = rightear - leftear
leftear2nose = nose - rightear
floorPlaneNormal = np.cross(leftear2ear, leftear2nose)

# now take eye 2 eye
lefteye2eye = righteye - lefteye
# face plane's normal:
facePlaneNormal = np.cross(lefteye2eye, floorPlaneNormal)

#finally make sure it's going forward not backward. 
# expect ear to nose to be going somewhat forward:
ear2nose = nose-leftear
if np.dot(ear2nose, lefteye2eye) < 0:
    # need to flip the vector
    facePlaneNormal = -1 * facePlaneNormal

# normalise
facePlaneNormal = facePlaneNormal / np.linalg.norm(facePlaneNormal)
print(facePlaneNormal)


# using planes, to be more general with the points.
# Extract the x, y, z coordinates
face_points = [nose, lefteye, righteye, leftear, rightear]
x_vals = np.array([p[0] for p in face_points])
y_vals = np.array([p[1] for p in face_points])
z_vals = np.array([p[2] for p in face_points])

# Create a matrix of input features (x, y)
X = np.column_stack((x_vals, y_vals))

# Fit a linear regression model to predict z based on x, y
model = LinearRegression()
model.fit(X, z_vals)

# Coefficients for the plane equation: Ax + By + D = z
A, B = model.coef_
C = -1
D = model.intercept_
#print(f"Plane equation: z = {A:.2f}x + {B:.2f}y + {D:.2f}")
floorPlaneNormal = np.array([A,B,C])

# now take eye 2 eye (!!! requires eye to eye.)
lefteye2eye = righteye - lefteye

# face plane's normal:
facePlaneNormal = np.cross(lefteye2eye, floorPlaneNormal)
