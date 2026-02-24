# SPDX-License-Identifier: ISC
#

from Basilisk.fswAlgorithms import sunlineSRuKFTemplated

def main():
    module = sunlineSRuKFTemplated.SunlineSRuKFTemplated()
    module.reset(0)
    module.updateState(0)

if __name__ == "__main__":
    main()
