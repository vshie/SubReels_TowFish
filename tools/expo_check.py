import numpy as np
# AP_Motors::apply_thrust_curve_and_volt_scaling, lift_max=1, batt scale=1
def actuator(thrust, expo):
    if expo == 0: return thrust
    return ((expo-1.0)+np.sqrt((1.0-expo)**2 + 4.0*expo*thrust))/(2.0*expo)

expo=0.65
print("MOT_THST_EXPO = 0.65 : what the depth controller asks for vs what the wing gets")
print(f"{'demand':>8s} {'servo travel':>13s} {'local gain':>11s}")
for t in (0.0,0.05,0.1,0.25,0.5,0.75,1.0):
    g=(actuator(t+1e-4,expo)-actuator(max(0,t-1e-4),expo))/(min(1,t+1e-4)-max(0,t-1e-4))
    print(f"{t*100:7.0f}% {actuator(t,expo)*100:12.1f}% {g:10.2f}x")
g0=1.0/np.sqrt((1-expo)**2); g1=1.0/np.sqrt((1-expo)**2+4*expo)
print(f"\nsmall-signal gain at trim = {g0:.2f}x, at full deflection = {g1:.2f}x"
      f"  ->  {g0/g1:.1f}:1 gain variation across the stroke")
print("for a servo, angle is linear in PWM and lift is linear in angle until stall,")
print("so the correct expo is 0. Non-zero expo makes the loop hottest exactly where")
print("the wing is most linear, and softest where it is about to stall.")

# stall knee in demand terms
knee_us=190.0; halfspan=(2001-1000)/2.0
print(f"\nstall knee at +{knee_us:.0f} us = {100*knee_us/halfspan:.0f}% of servo travel")
# invert: what demand produces that travel?
from scipy.optimize import brentq
dem=brentq(lambda x: actuator(x,expo)-knee_us/halfspan, 0, 1)
print(f"  reached at only {dem*100:.0f}% of commanded thrust with expo=0.65")
print(f"  would need {100*knee_us/halfspan:.0f}% of commanded thrust with expo=0")
