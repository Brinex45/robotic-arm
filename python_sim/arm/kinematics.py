import numpy as np
import math as m

matrix = np.array([
    [0, 0, 1, 32.9],
    [0, 1, 0, 0],
    [-1, 0, 0, 6],
    [0, 0, 0, 1]
])

nx1 = matrix[0,0]
ny1 = matrix[1,0]
nz1 = matrix[2,0]

sx1 = matrix[0,1]
sy1 = matrix[1,1]
sz1 = matrix[2,1]

ax1 = matrix[0,2]
ay1 = matrix[1,2]
az1 = matrix[2,2]

Px1 = matrix[0,3]
Py1 = matrix[1,3]
Pz1 = matrix[2,3]

d1_test = 6
d5_test = 16
a2_test = 10.4
a3_test = 6.5

t1 = m.degrees(m.atan2(Py1, Px1))
t234 = m.degrees(m.atan2(-( (ax1 * m.cos(m.radians(t1))) + (ay1 * m.sin(m.radians(t1)) )),-az1))

#t5 = (m.atan2((nx1*m.sin(m.radians(t1)) - ny1*m.cos(m.radians(t1))), (sx1*m.sin(m.radians(t1)) - sy1*m.cos(m.radians(t1)))))
t5 = m.degrees( m.atan2(sz1, -nz1) )
c = (Px1/m.cos(m.radians(t1))) + d5_test*m.sin(m.radians(t234))
d = d1_test - d5_test*m.cos(m.radians(t234)) - Pz1

costheta3 = (c**2 + d**2 - a2_test**2 - a3_test**2) / (2 * a3_test * a2_test)
costheta3 = min(max(costheta3, -1), 1)
sintheta3 = m.sqrt(1 - costheta3**2)

t3 = m.degrees(m.atan2(sintheta3,costheta3))

r = a3_test*m.cos(m.radians(t3)) + a2_test
s = a3_test*m.sin(m.radians(t3))

t2 = m.degrees(m.atan2(r*d - s*c, r*c + s*d))
t4 = t234 - t3 - t2 + 180

print(f"t234: {t234:.2f}°")
print(f"c: {c:.2f}°")
print(f"d: {d:.2f}°")
print(f"costheta3: {costheta3:.2f}°")
print(f"sintheta3: {sintheta3:.2f}°")
print(f"r: {r:.2f}°")
print(f"s: {s:.2f}°")
print("\n")

print(f"Theta 1: {t1:.2f}°")
print(f"Theta 2: {t2:.2f}°")
print(f"Theta 3: {t3:.2f}°")
print(f"Theta 4: {t4:.2f}°")
print(f"Theta 5: {t5:.2f}°")