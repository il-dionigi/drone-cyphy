# CyPhy-Drone

Work done as part of the Cyber Physical lab under Prof. Tabuada and PhD candidate Omar Hussien during summer 2018. Contributed to by Tameez Latib, Joey Miller, and Brian Raymond.

**Notes**

To get radio working for addresses other than the default *0xE7E7E7E7E7*, such as *0xE7E7E7E7E8*, make the following change in crazyflie-lib-python/cflib/drivers styled like:

    self.set_address((0xE7,)*4 + (0xE8,)) //crazyradio.py:141

