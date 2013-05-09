
(in-package :asdf)

(defsystem "asctec-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "Altitude" :depends-on ("_package"))
    (:file "_package_Altitude" :depends-on ("_package"))
    (:file "DMCmd" :depends-on ("_package"))
    (:file "_package_DMCmd" :depends-on ("_package"))
    (:file "HWCmd" :depends-on ("_package"))
    (:file "_package_HWCmd" :depends-on ("_package"))
    (:file "PDCmd" :depends-on ("_package"))
    (:file "_package_PDCmd" :depends-on ("_package"))
    (:file "PelicanStatus" :depends-on ("_package"))
    (:file "_package_PelicanStatus" :depends-on ("_package"))
    (:file "PressureSensor" :depends-on ("_package"))
    (:file "_package_PressureSensor" :depends-on ("_package"))
    (:file "SerialStatus" :depends-on ("_package"))
    (:file "_package_SerialStatus" :depends-on ("_package"))
    (:file "Voltage" :depends-on ("_package"))
    (:file "_package_Voltage" :depends-on ("_package"))
    ))
