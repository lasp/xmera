%module rigidBodyKinematics
%{
  #include "rigidBodyKinematics.hpp"
%}

%include "std_string.i"
%include "std_vector.i"
%include "swig_conly_data.i"
%include "swig_eigen.i"
%include "rigidBodyKinematics.hpp"

// Instantiate all functions for float and double where the
// double instantiations are given under existing function names
%template(mrpShadow) mrpShadow<double>;
%template(mrpShadowFloat) mrpShadow<float>;

%template(mrpSwitch) mrpSwitch<double>;
%template(mrpSwitchFloat) mrpSwitch<float>;

%template(binvEp) binvEp<double>;
%template(binvEpFloat) binvEp<float>;

%template(binvMrp) binvMrp<double>;
%template(binvMrpFloat) binvMrp<float>;

%template(binvPrv) binvPrv<double>;
%template(binvPrvFloat) binvPrv<float>;

%template(binvEulerAngles321) binvEulerAngles321<double>;
%template(binvEulerAngles321Float) binvEulerAngles321<float>;

%template(bmatEp) bmatEp<double>;
%template(bmatEpFloat) bmatEp<float>;

%template(bmatMrp) bmatMrp<double>;
%template(bmatMrpFloat) bmatMrp<float>;

%template(bmatDotMrp) bmatDotMrp<double>;
%template(bmatDotMrpFloat) bmatDotMrp<float>;

%template(bmatPrv) bmatPrv<double>;
%template(bmatPrvFloat) bmatPrv<float>;

%template(bmatEulerAngles321) bmatEulerAngles321<double>;
%template(bmatEulerAngles321Float) bmatEulerAngles321<float>;

%template(dcmToEp) dcmToEp<double>;
%template(dcmToEpFloat) dcmToEp<float>;

%template(dcmToMrp) dcmToMrp<double>;
%template(dcmToMrpFloat) dcmToMrp<float>;

%template(epToPrv) epToPrv<double>;
%template(epToPrvFloat) epToPrv<float>;

%template(dcmToPrv) dcmToPrv<double>;
%template(dcmToPrvFloat) dcmToPrv<float>;

%template(dcmToEulerAngles321) dcmToEulerAngles321<double>;
%template(dcmToEulerAngles321Float) dcmToEulerAngles321<float>;

%template(dep) dep<double>;
%template(depFloat) dep<float>;

%template(dmrp) dmrp<double>;
%template(dmrpFloat) dmrp<float>;

%template(dmrpToOmega) dmrpToOmega<double>;
%template(dmrpToOmegaFloat) dmrpToOmega<float>;

%template(ddmrp) ddmrp<double>;
%template(ddmrpFloat) ddmrp<float>;

%template(ddmrpTodOmega) ddmrpTodOmega<double>;
%template(ddmrpTodOmegaFloat) ddmrpTodOmega<float>;

%template(dprv) dprv<double>;
%template(dprvFloat) dprv<float>;

%template(deuler321) deuler321<double>;
%template(deuler321Float) deuler321<float>;

%template(epToDcm) epToDcm<double>;
%template(epToDcmFloat) epToDcm<float>;

%template(epToMrp) epToMrp<double>;
%template(epToMrpFloat) epToMrp<float>;

%template(epToEulerAngles321) epToEulerAngles321<double>;
%template(epToEulerAngles321Float) epToEulerAngles321<float>;

%template(tildeMatrix) tildeMatrix<double>;
%template(tildeMatrixFloat) tildeMatrix<float>;

%template(mrpToDcm) mrpToDcm<double>;
%template(mrpToDcmFloat) mrpToDcm<float>;

%template(mrpToEp) mrpToEp<double>;
%template(mrpToEpFloat) mrpToEp<float>;

%template(mrpToPrv) mrpToPrv<double>;
%template(mrpToPrvFloat) mrpToPrv<float>;

%template(mrpToEulerAngles321) mrpToEulerAngles321<double>;
%template(mrpToEulerAngles321Float) mrpToEulerAngles321<float>;

%template(prvToDcm) prvToDcm<double>;
%template(prvToDcmFloat) prvToDcm<float>;

%template(prvToEp) prvToEp<double>;
%template(prvToEpFloat) prvToEp<float>;

%template(prvToMrp) prvToMrp<double>;
%template(prvToMrpFloat) prvToMrp<float>;

%template(prvToEulerAngles321) prvToEulerAngles321<double>;
%template(prvToEulerAngles321Float) prvToEulerAngles321<float>;

%template(eulerAngles321ToDcm) eulerAngles321ToDcm<double>;
%template(eulerAngles321ToDcmFloat) eulerAngles321ToDcm<float>;

%template(eulerAngles321ToEp) eulerAngles321ToEp<double>;
%template(eulerAngles321ToEpFloat) eulerAngles321ToEp<float>;

%template(eulerAngles321ToMrp) eulerAngles321ToMrp<double>;
%template(eulerAngles321ToMrpFloat) eulerAngles321ToMrp<float>;

%template(eulerAngles321ToPrv) eulerAngles321ToPrv<double>;
%template(eulerAngles321ToPrvFloat) eulerAngles321ToPrv<float>;

%template(addEp) addEp<double>;
%template(addEpFloat) addEp<float>;

%template(addEulerAngles321) addEulerAngles321<double>;
%template(addEulerAngles321Float) addEulerAngles321<float>;

%template(addPrv) addPrv<double>;
%template(addPrvFloat) addPrv<float>;

%template(addMrp) addMrp<double>;
%template(addMrpFloat) addMrp<float>;

%template(subEp) subEp<double>;
%template(subEpFloat) subEp<float>;

%template(subMrp) subMrp<double>;
%template(subMrpFloat) subMrp<float>;

%template(subPrv) subPrv<double>;
%template(subPrvFloat) subPrv<float>;

%template(subEulerAngles321) subEulerAngles321<double>;
%template(subEulerAngles321Float) subEulerAngles321<float>;

%template(rotationMatrix) rotationMatrix<double>;
%template(rotationMatrixFloat) rotationMatrix<float>;
