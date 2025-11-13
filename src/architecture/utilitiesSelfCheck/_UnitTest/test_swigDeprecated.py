import pytest

from xmera.architecture import swigDeprecatedCheck
from xmera.utilities import deprecated


def get_class_without_warning():
    with deprecated.catch_warnings():
        deprecated.filterwarnings(
            "ignore", "SwigDeprecatedTestClass.SwigDeprecatedTestClass"
        )
        return swigDeprecatedCheck.SwigDeprecatedTestClass()


def test_test1():
    """Checks that stand-alone functions generates appropriate warnings"""
    with pytest.warns(deprecated.BSKUrgentDeprecationWarning, match="(.*)test1 Msg"):
        swigDeprecatedCheck.test1(0, 0)


def test_class():
    """Checks that instantiating classes generates appropriate warnings"""
    with pytest.warns(deprecated.BSKDeprecationWarning, match="(.*)class Msg"):
        swigDeprecatedCheck.SwigDeprecatedTestClass()


def test_test2():
    """Checks that calling class methods generates appropriate warnings"""
    testClass = get_class_without_warning()
    with pytest.warns(deprecated.BSKUrgentDeprecationWarning, match="(.*)test2 Msg"):
        testClass.test2()


def test_test3():
    """Checks that calling class methods generates appropriate warnings"""
    testClass = get_class_without_warning()
    with pytest.warns(deprecated.BSKDeprecationWarning, match="(.*)test3 Msg"):
        testClass.test3()


def test_test4_set():
    """Checks that setting deprecated variables generates appropriate warnings"""
    testClass = get_class_without_warning()
    with pytest.warns(deprecated.BSKDeprecationWarning, match="(.*)test4 Msg"):
        testClass.test4 = 1

        assert testClass.test4 == 1


def test_test4_get():
    """Checks that calling deprecated variables generates appropriate warnings"""
    testClass = get_class_without_warning()
    with pytest.warns(deprecated.BSKDeprecationWarning, match="(.*)test4 Msg"):
        _ = testClass.test4


if __name__ == "__main__":
    swigDeprecatedCheck.test1(0, 0)
    test = swigDeprecatedCheck.SwigDeprecatedTestClass()
    test.test2()
    test.test3()
    test.test4 = 5
    print(test.test4)
