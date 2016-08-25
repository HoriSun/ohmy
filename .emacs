(custom-set-variables
  ;; custom-set-variables was added by Custom.
  ;; If you edit it by hand, you could mess it up, so be careful.
  ;; Your init file should contain only one such instance.
  ;; If there is more than one, they won't work right.
 '(inhibit-startup-screen t)
 '(global-linum-mode 1) ;; add number at beginning of each line
 '(global-hl-line-mode 1) ;; highlight current line

)
(custom-set-faces
  ;; custom-set-faces was added by Custom.
  ;; If you edit it by hand, you could mess it up, so be careful.
  ;; Your init file should contain only one such instance.
  ;; If there is more than one, they won't work right.
 '(hl-line ((default nil) (nil (:background "gray25"))))
)

(defvar first-time t
  "Flag signifying this is the first time that .emacs has been evaled")

;; xml file format coloring
;;(setq auto-mode-alist (cons '("\\.launch$" . xml-mode) auto-mode-alist))  
;;(setq auto-mode-alist (cons '("\\.xacro$" . xml-mode) auto-mode-alist))  
;;(setq auto-mode-alist (cons '("\\.world$" . xml-mode) auto-mode-alist))  
;;(setq auto-mode-alist (cons '("\\.gazebo$" . xml-mode) auto-mode-alist))
;;(setq auto-mode-alist (cons '("\\.urdf$" . xml-mode) auto-mode-alist))  
(if first-time
    (setq auto-mode-alist
      (append '(("\\.cpp$" . c++-mode)
                ("\\.hpp$" . c++-mode)
                ("\\.lsp$" . lisp-mode)
                ("\\.scm$" . scheme-mode)
                ("\\.pl$" . perl-mode)
                ("\\.launch$" . xml-mode)
                ("\\.xacro$" . xml-mode) 
                ("\\.world$" . xml-mode)
                ("\\.gazebo$" . xml-mode)
                ("\\.urdf$" . xml-mode)
                ("\\.wsgi$" . python-mode)
                ("\\.yaml$" . conf-mode)
                ) auto-mode-alist)))
;;(colum-number-mode "1")
;;(setq tab-width 4)  ;; tab width 4
;;(setq default-indent-width 4)  ;; tab width 4
(set-background-color "gray11")
(set-face-background 'region "RoyalBlue4")

(setq-default indent-tabs-mode nil)
(setq-default tab-width 4)
(dolist (hook (list                     ;设置用空格替代TAB的模式
               'emacs-lisp-mode-hook
               'lisp-mode-hook
               'lisp-interaction-mode-hook
               'scheme-mode-hook
               'c-mode-hook
               'c++-mode-hook
               'java-mode-hook
               'haskell-mode-hook
               'asm-mode-hook
               'emms-tag-editor-mode-hook
               'sh-mode-hook
               ))
  (add-hook hook '(lambda () (setq indent-tabs-mode nil)))
  (add-hook hook '(lambda () (setq tab-width 4))))
;;(setq indent-line-function 'insert-tab)

;;(set-foreground-color "white smoke")
(set-foreground-color "gray80")
(put 'upcase-region 'disabled nil)




(setq first-time nil)
