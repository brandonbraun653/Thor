from pystlink import PyStlink
from lib import dbg


if __name__ == '__main__':

    test = PyStlink()
    test._dbg = dbg.Dbg(1)
    test.detect_cpu(expected_cpus=['STM32F765xI', 'STM32F767xI', 'STM32F769xI',
                                   'STM32F777xI', 'STM32F778xI', 'STM32F779xI'])
    #test.start()

    try:
        test.cmd(['flash', 'erase', 'verify', '0x08000000', 'ChimeraDevelopment.bin'])

    except:
        pass

    test.cmd(['write', '0x08000000', 'ChimeraDevelopment.bin'])
    test.cmd(['reset'])
