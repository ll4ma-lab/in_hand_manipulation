(TeX-add-style-hook "point_contact"
 (lambda ()
    (LaTeX-add-labels
     "sec:ik-rigid"
     "sec:point-contact")
    (TeX-run-style-hooks
     "amsmath"
     "mathtools"
     "times"
     "latex2e"
     "art10"
     "article"
     "")))

