# CyPhy-Drone

Work done as part of the Cyber Physical lab under Prof. Tabuada and PhD candidate Omar Hussien during summer 2018. Contributed to by Tameez Latib, Joey Miller, and Brian Raymond.

**Notes**

To get radio working for addresses other than the default *0xE7E7E7E7E7*, such as *0xE7E7E7E7E8, make the following change in crazyflie-lib-python/cflib styled like:

    self.set_address((0xE7,)*4 + (0xE8,)) //drivers/crazyradio.py:141
    DEFAULT_ADDR_A = [0xe7, 0xe7, 0xe7, 0xe7, 0xe8] //crtp/radiodriver.py:61
    DEFAULT_ADDR = 0xE7E7E7E7E8 //crtp/radiodriver.py:62