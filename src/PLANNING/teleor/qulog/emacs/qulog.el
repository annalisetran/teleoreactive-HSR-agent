;; qulog.el - major mode for Qulog
;; Copyright (C) 2016 Keith Clark, Peter Robinson
;; Email: klc@doc.ic.ac.uk, pjr@itee.uq.edu.au

;; This library is free software; you can redistribute it and/or modify
;; it under the terms of the GNU General Public License as published by
;; the Free Software Foundation; either version 2 of the License, or
;; (at your option) any later version.

;; This library is distributed in the hope that it will be useful,
;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;; GNU General Public License for more details.

;; You should have received a copy of the GNU General Public License
;; along with this program; if not, write to the Free Software
;; Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


(require 'font-lock) 



;; (electric-pair-mode 1)

;; (setq electric-pair-pairs
;;       '(
;;         (?\" . ?\")
;;         (?\[ . ?\])
;;         (?\{ . ?\})
;;         ))

;; get rid of all tabs on startup
(defvar qulog-mode-hook (lambda () (untabify (point-min) (point-max))))

;; make sure syntax highlighting is turned on
(add-hook 'qulog-mode-hook '(lambda () (font-lock-mode 1)))
;(add-hook 'qulog-mode-hook '(lambda () (electric-indent-mode 1)))

(defvar qulog-mode-map
  (let ((map (make-keymap)))
    (define-key map "\C-j" 'newline-and-indent)  
    (define-key map ")" 'close-brac-and-indent)
    (define-key map "]" 'close-sbrac-and-indent)
    (define-key map "}" 'close-paren-and-indent)
    (define-key-after map [menu-bar] (make-sparse-keymap) 'tools)
    map)
  "Keymap for qulog major mode")

(let ((menuMap (make-sparse-keymap "Qulog")))
  (define-key qulog-mode-map [menu-bar ql] (cons "Qulog" menuMap))
  (define-key menuMap [commentout]
    '("Comment Region" . comment-region))
  (define-key menuMap [uncommentout]
    '("Uncomment Region" . uncomment-region))
  )

(defun close-brac-and-indent ()
  (interactive)
  (insert ")")
  (qulog-indent-line) 
  )

(defun close-sbrac-and-indent ()
  (interactive)
  (insert "]")
  (qulog-indent-line) 
  )

(defun close-paren-and-indent ()
  (interactive)
  (insert "}")
  (qulog-indent-line) 
  )

(defun qulog-newline-and-indent ()
  (interactive)
  (insert "\n ")
  (qulog-indent-line) 
  )

(defun set-newline-and-indent ()
  (local-set-key (kbd "RET") 'qulog-newline-and-indent ) 
  (local-set-key (kbd "<S-return>") 'newline-and-indent ))


(defvar qulog-mode-syntax-table
  (let ((qulog-mode-syntax-table (make-syntax-table)))
    (modify-syntax-entry ?* ". 23b" qulog-mode-syntax-table)
    (modify-syntax-entry ?/ ". 14" qulog-mode-syntax-table)
    (modify-syntax-entry ?% "<" qulog-mode-syntax-table)
    (modify-syntax-entry ?\n ">" qulog-mode-syntax-table)
    (modify-syntax-entry ?_ "w" qulog-mode-syntax-table)
    qulog-mode-syntax-table)
  "Syntax table for qulog-mode")

(defun qulog-comment-dwim (arg)
  "Comment or uncomment current line or region in a smart way."
  (interactive "*P")
  (require 'newcomment)
  (let (
        (comment-start "%") (comment-end "")
        )
    (comment-dwim arg)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; management of faces

;; for "" strings
(custom-set-faces '(font-lock-string-face ((t (:foreground "#006400")))))
;; for '' quoted atoms
(custom-set-faces '(font-lock-doc-face ((t (:foreground "#6b8e23")))))
;; for keywords (was "#eb00db")
(custom-set-faces '(font-lock-keyword-face ((t (:foreground "#9400d3")))))
;; for builtins
;;(custom-set-faces '(font-lock-builtin-face ((t (:foreground "#9400d3")))))
;; for variables (was "#aa6052") "#705000"
(custom-set-faces '(font-lock-variable-name-face ((t (:foreground "#855000")))))
;; for numbers
(custom-set-faces '(font-lock-constant-face ((t (:foreground "#316994")))))

(custom-set-faces '(font-lock-preprocessor-face ((t (:background "#e579f4")))))


;; keywords
(defconst qulog-keywords 
  (list "once"  "or_while" "commit_while" "min_time" 
        "forall" "not"  "wait_case" "watch" "timeout" "retry" 
	"to" "in" "exists" "union" "inter" "diff" 
        "false" "true" 
        "from" "from_thread"
        "receive" "case" "try" "except" 
        "repeat" "until"
        "min_time" "default" "query_at" "of" 
        "atomic_action"))

 
(defconst qulog-keywords-face-regexp
   (concat "\\<" (regexp-opt qulog-keywords t) "\\>"))

;; (defconst qulog-keywords-face-regexp
;;   (concat  "\\(^\\|[^_a-zA-Z0-9]\\)"
;;            (regexp-opt qulog-keywords t) "\\>"))

(defconst qulog-decl-keywords
   (list "nondet act" "rel" "act" "fun" "tel" "dyn" "mrel" "mfun" "def"
         "tel_percept" "tel_action" "tel_start" "tel_atomic" "int" "num" )
  )

(defconst qulog-decl-keywords-face-regexp
  (concat "\\(^\\)" 
	  (regexp-opt qulog-decl-keywords t)
	  "\\([ ]\\)")
  )

			 

;; NOTE: "\\(\\\\/\\|[#$/]\\\\\\)" is for # $ /\ \/

(defconst qulog-font-lock-keywords-1
  (list
   ;;(list "\\(<<<\\)" 1 font-lock-preprocessor-face)
   ;;(list "\\(>>>\\)" 1 font-lock-preprocessor-face)
   ;; quoted atoms
   (list "\\(^\\|[^0-9]\\)\\('\\([^\n']\\|\\\\'\\)*'\\)" 2 font-lock-doc-face)
   ;; builtins
   ;;(list qulog-builtins-face-regexp 2 font-lock-builtin-face)
   ;; keywords
   (list qulog-keywords-face-regexp 1 font-lock-keyword-face)
   (list qulog-decl-keywords-face-regexp 2 font-lock-keyword-face)

   ;; more keywords ( # $ /\ \/ )
   (list "\\(\\\\/\\|[#$/]\\\\\\)" 1 font-lock-keyword-face)
   ;; numbers
   (list "[^_a-zA-Z0-9]\\(\\(\\+\\|-\\)?[0-9]+\\(\\.[0-9]+\\)?\\)" 
         1 font-lock-constant-face)
   ;; functor at head of rule
   (list "^\\([a-z][A-Za-z0-9_]*\\)" 1 font-lock-function-name-face)
   ;; variables - word starts with upper case or _
   (list "\\<\\([A-Z_][A-Za-z0-9_]*\\)" 1 font-lock-variable-name-face)

   )
  "Highlights")

(defvar qulog-font-lock-keywords qulog-font-lock-keywords-1
  "Default highlighting expressions")


(defun qulog-mode (&optional system)
  (interactive)
  (kill-all-local-variables)
  (use-local-map qulog-mode-map)
  (set-syntax-table qulog-mode-syntax-table)
  (setq major-mode 'qulog-mode)
  (setq font-lock-defaults '(qulog-font-lock-keywords))
  (define-key qulog-mode-map [remap comment-dwim] 'qulog-comment-dwim)
  (setq mode-name "Qulog")
  (setq use-empty-active-region nil)
  (setq qulog-indent-level 4)
  (setq indent-tabs-mode nil)
  (show-paren-mode 1)
  (make-local-variable 'comment-start)
  (setq comment-start "%")
  (make-local-variable 'comment-end)
  (setq comment-end "")
  (set-newline-and-indent)
  (set (make-local-variable 'indent-line-function) 'qulog-indent-line)
  (run-mode-hooks 'qulog-mode-hook)
  )

(defconst indent-keywords 
  (list "<=" "~>" "~+>" "->" "::" "::=" "==" "=>" "??" 
        ))

(defconst indent-infix-operators 
  (list  "=?" "=" "=@"
	":=" "+:=" "-:="  "@<" "@=" "@=<" "@>" "@>=" 
	"\\=" 
	">=" "<" "=<" ">" 
	 "or_while" "commit_while" "min_time" "of"
        ))

(defconst infix-operators 
  (list  "<<" ">>"  "\\"
	 "+" "-" "**"  "/" "//" "\\/" "/\\" "*" "mod" 
         "to_thread" "to" "from" "from_thread" "with"
         "++" "<>" "++?" "<>?"
         "except"
         "@.." "until"
         "in" "retry" ":"  "at" "@" "query_at" 
        ))

(defconst prefix-operators 
  (list "not" "once" "call" "do" "doTR" "?" "try" "repeat" "receive"
        )
  )

(defconst quantifier (list "forall" "exists"))

(defconst qulog-indent-keywords-regexp
  (regexp-opt indent-keywords t))

(defconst qulog-prefix-operators-regexp
  (regexp-opt prefix-operators t))

(defconst qulog-infix-operators-regexp
  (regexp-opt infix-operators t))

(defconst qulog-indent-infix-operators-regexp
  (regexp-opt indent-infix-operators t))

(defconst continuation-operators
  (list "," ";"  ".."  "&" "|" "||" ))

(defconst open-bracs (list "{" "(" "[" ))

(defconst defn-operators
  (list "rel" "act" "dyn" "mrel" "mfun" "fun" "tel" "tel_start" "tel_atomic" "tel_percept" "tel_action" "def"))

(defconst qulog-continuation-operators-regexp
  (regexp-opt continuation-operators t))

(defconst qulog-open-brac-regexp
  (regexp-opt '("{" "(" "[" ) t))

(defconst qulog-close-brac-regexp
  (regexp-opt '("}" ")" "]" ) t))
 
(defconst qulog-string-regexp "\"\\(\\(?:[^\"]+\\|\\\\\\(?:.\\|\\\n\\|\\)\\)*\\)\"")

(defconst qulog-quote-regexp "'\\(\\(?:[^'\]+\\|\\\\\\(?:.\\|\\\n\\|\\)\\)*\\)'")

(defconst qulog-number-regexp 
  "\\([0-9]+\\(\\.[0-9]+\\)?\\)")

;;  (regexp-opt '("[_a-zA-Z][_a-zA-Z0-9]*"))
(defvar qulog-name-regexp
  "\\([_a-zA-Z][_a-zA-Z0-9]*\\)")

(defvar qulog-other-regexp
  "\\([^ \t\n]\\)")
  
(defconst context-pop-end
  (list "NL START"))


(defun qulog-indent-line (&optional whole-exp)
  (if (bobp) 
    (indent-line-to 0)
    (let ((indent (compute-indent-level)))
      ;; the following keeps track of where cursor was so 
      ;; stays there after indentation
      (let ((opoint (point-marker)))
	(indent-line-to indent)
	(if (> opoint (point))
	    (goto-char opoint))
	(set-marker opoint nil))
      )
    )
  )

(defun qulog-matched-bracket (open close)
  ;(message "qulog-matched-bracket %S %S" open close)
  (or
   (and (string-equal open "(") (string-equal close ")"))
   (and (string-equal open "{") (string-equal close "}"))
   (and (string-equal open "[") (string-equal close "]"))
   ;;(and (string-equal open ">>>") (string-equal close "<<<"))
   )
  )
;;
;; a token is  (cons kind text)
;; where text is the text of the token and kind is the kind of token
;;
;; context is a list whose entries are
;; (list token indent)
;; where token is the token at this point and indent is the current
;; indent for this token 
;;


(defun compute-indent-level ()
  (interactive)
  (save-excursion
    (beginning-of-line)
    (setq line_pos (point))
    (back-to-previous-start)
    ;; initialize the context with a dummy node
    (setq context (cons (cons (cons "NL START" "NL START") 0) nil))
    (setq curr-indent 0) 
    (setq token (cons "" ""))
    (setq prev-is-op nil)
    (while (< (point) line_pos)
      (setq token (next-token))
      (setq curr-indent (cdr (car context)))
      
      (setq current-line (line-number-at-pos))
      (setq begin (line-beginning-position))
      ;;(message "begin %S line_pos %S" begin line_pos) 
      (when (> begin line_pos)
        (setq token (cons "default" "default")))
      ;;(message  "point %S line_pos %S token %S indent %s prev is op %S context %S " (point) line_pos token curr-indent prev-is-op context)
      
      (cond
       
       (
        (string-equal (car token) "defn-operator")
        (setq curr-indent qulog-indent-level)
        (setq context
              (cons (cons token curr-indent) context))
        (setq prev-is-op t)
        )
       
       ;; 
       (
        (string-equal (car token) "continuation-operators")
        (setq context (pop-context token  context))
        (setq context (cons (cons token (head-token-indent context)) context))
        (setq curr-indent (cdr (car context)))
        (setq prev-is-op t)
        )

       (
        (string-equal (car token) "indent-operator")
        (setq context (pop-context token  context))
        (setq curr-indent (+ (head-token-indent context)  qulog-indent-level))
        (if (at-start-of-line (cdr token))
            (setq context (cons (cons token
                                      (+ curr-indent qulog-indent-level))
                                context))
          (setq context (cons (cons token curr-indent) context))
          )
        (setq prev-is-op t)
        )

       (
        (and (string-equal (cdr token) "++")
             (name-in-context "~>" context))
        (setq context (pop-context token  context))
        (setq curr-indent (+ (head-token-indent context)  qulog-indent-level))
        (if (at-start-of-line (cdr token))
            (setq context (cons (cons token
                                      (+ curr-indent qulog-indent-level))
                                context))
          (setq context (cons (cons token curr-indent) context))
          )
        (setq prev-is-op t)
        )

       (
        (and (string-equal (cdr token) "until") (at-start-of-line (cdr token)))
        (setq context (pop-context token  context))
        (setq context (cons (cons token (head-token-indent context)) context))
        (setq curr-indent (- (cdr (car context)) qulog-indent-level))
        (setq prev-is-op t)
        
        )
       
       (
        (string-equal (car token) "infix-operator")
        (setq context (pop-context token  context))
        (setq context (cons (cons token (head-token-indent context)) context))
        (setq curr-indent (cdr (car context)))
        (setq prev-is-op t)
        )
       
       (
        (string-equal (car token) "indent-infix-operator")
        (setq context (pop-context token  context))
        (setq curr-indent (+ (head-token-indent context)  qulog-indent-level))
        (if (at-start-of-line (cdr token))
            (setq context (cons (cons token
                                      (+ curr-indent qulog-indent-level))
                                context))
          (setq context (cons (cons token curr-indent) context))
          )
        (setq prev-is-op t)
        )
       
       (
        (string-equal (car token) "prefix-operator")
        (when (not prev-is-op)
          (if (name-in-context "bound" context)
              (progn
                (while (not (string-equal "bound" (car (head-token context))))
                  (setq context (cdr context)))
                (setq context (cdr context))
                )
            (setq head (head-token context))
            (while (and (not (string-equal (car head) "openbrac")) 
                        (not (member (car head) context-pop-end)))
              (setq context (cdr context))
              (setq head (head-token context))
              )
            )
          )
        (setq curr-indent (cdr (car context)))
        (setq context (pop-context token  context))
        (setq context (cons (cons token
                                  (+ (head-token-indent context)
                                     qulog-indent-level)) context))
        (setq prev-is-op t)
        )
       
       (
        (string-equal (car token) "quantifier")
        (setq context 
              (cons (cons token (+ curr-indent  qulog-indent-level)) context))
        (setq context 
              (cons (cons (cons "bound" "bound") (+ 1 (- (point) begin)))
                    context))
        
        )

       (
        (and (string-equal (car token) "openbrac")
             (or
              (string-equal "repeat" (cdr (head-token context)))
              (string-equal "try" (cdr (head-token context)))
              (string-equal "receive" (cdr (head-token context)))
             (string-equal "until" (cdr (head-token context)))))
        (cond
         (
          (no-more-tokens-on-line)
          (setq context 
                (cons (cons token (head-token-indent context))
                      context))
          )
         (
          t
          (setq context 
                (cons (cons token  (- (point) begin))
                      context))
          )
         )
              
        )
              
       (
        (string-equal (car token) "openbrac")       
        (setq prev-is-op t)
        (when (name-in-context "bound" context)
          (while (not (string-equal "bound" (car (head-token context))))
            (setq context (cdr context)))
          (setq context (cdr context))
          )
        (cond
         (
          (no-more-tokens-on-line)
          (setq context 
                (cons (cons token (+  (head-token-indent context)
                               qulog-indent-level))
                      context))
          )
         (
          t
          (setq context 
                (cons (cons token  (- (point) begin))
                      context))
          )
         )
        )
       
       (
        (string-equal (car token) "closebrac")
        (setq prev-is-op nil)
        (setq head (head-token context))
        (while (and (not (string-equal (car head) "openbrac")) 
                    (not (member (car head) context-pop-end)))
          (setq context (cdr context))
          (setq head (head-token context))
          )
        
        (if (qulog-matched-bracket (cdr (head-token context)) (cdr token))
            (progn
              (setq curr-indent (cdr (car context)))
              ;;(when (string-equal (cdr token) "<<<")
              ;;  (setq curr-indent (- curr-indent qulog-indent-level)))
              (setq context (cdr context))
              )
          (setq curr-indent 0)
          (message "Unmatched bracket")
          (setq line_pos (point))
          )
        )

       
       (
        t
        (when (not prev-is-op)
          (if (name-in-context "bound" context)
              (progn
                (while (not (string-equal "bound" (car (head-token context))))
                  (setq context (cdr context)))
                (setq context (cdr context))
                )
            (setq head (head-token context))
            (while (and (not (string-equal (car head) "openbrac")) 
                        (not (member (car head) context-pop-end)))
              (setq context (cdr context))
              (setq head (head-token context))
              )
            )
          )
        (setq curr-indent (cdr (car context)))
        (setq prev-is-op nil)
        )
       )
      )     
     ;;(message "END : indent %S  context %S" curr-indent context)
    curr-indent
    )
  )


;;   precedence from lowest to highest (:: special case)
;;
;;   def rel act tel fun
;;   <= -> ~> ::= => ==
;;   as
;;   ::  (normal case)
;;   & ; ,
;;   not once call do doTR 
;;   = =? =@ \= := +:= -:=  @< @= @=< @> @>= >= < =< in
;;  ++ <>
;;  ::   (special case)
;;  + - \/ /\
;;  >> << mod 
;;  * / //
;;  **
;;  -  (as prefix)

(defun precedence-number (op)
  (cond 
   ((member op (list "def" "rel" "nondet act" "act" "tel" "fun" "tel_percept" "tel_action" "dyn" "mrel" "mfun")) 10)
   ((member op (list "commit_while")) 12)
   ((member op (list "or_while")) 14)
   ((member op (list "min_time")) 16)
   ((member op (list "<=" "->" "~>" "~+>" "::=" "=>" "==" "??" )) 20)
   ((member op (list ";" )) 40)
   ((member op (list "repeat")) 41)
   ((member op (list  "bound" "until"
                     "try" "except" "case" "wait" "wait_case"
                     "atomic_action" )) 42)
   ((member op (list ",")) 44)
   ((member op (list "?" )) 45)
   ;;((member op (list   "<>?" "++?")) 45)
   ((member op (list "&" )) 46)
   ((member op (list "forall" "exists" "remote_query")) 47)
   ((member op (list "not" "once" "call" "do" "doTR" "of" "at")) 50)
   ((member op (list  "=" "=@" "\\=" ":=" "+:=" "-:=" "=?"
                "@<" "@=" "@=<" "@>" "@>=" ">=" "<" ">" "=<" "in" "retry")) 60)
   ((member op (list  "++" "<>" "@.." "@"  "<>?" "++?")) 70)
   ((member op (list  "+" "-" "\\/" "/\\" )) 80)
   ((member op (list  ">>" "<<" "mod")) 90)
   ((member op (list   "*" "/" "//")) 100)
   ((member op (list  "**" ":")) 110)
   (t 999)
   )
  )


(defun has-ge-precedence (op1 op2 context)
  ;;(message "higher prec %S %S %S"  op1 op2 context)
  (cond
   ( (and (string-equal op1 ",") (string-equal op2 "->"))
     nil )
   ((string-equal op2 "NL START")
    (not (string-equal op1 "NL START"))
    )
   ((and (string-equal op2 "::") (string-equal op1 "&")
         (eqq-in-context context))
    t
    )
   ((string-equal op2 "::")
    (if (eqq-in-context context)
        (>= (precedence-number op1) 80)
      (>= (precedence-number op1) 20)
      )
    )
   ((string-equal op1 "::")
    (if (eqq-in-context context)
        (>= 80 (precedence-number op2))
      (>= 20 (precedence-number op2))
      )
    )
   ((and (string-equal op2 "++")
         (name-in-context "~>" context))
    (>= (precedence-number op1) 20)
    )
   ((and (string-equal op1 "++")
         (name-in-context "~>" context))
    (>= 20 (precedence-number op2))
    )
   (t
    (>= (precedence-number op1) (precedence-number op2))
    )
   )
  )

(defun pop-context (token  context)
  (if (or (has-ge-precedence (cdr token) (cdr (head-token context)) context)
          (string-equal (car (head-token context)) "openbrac"))
      context
    (pop-context token (cdr context))
    )
  )

(defun head-token (context)
   (car (car context))
   )
(defun head-token-indent (context)
  (cdr (car context))
  )


(defun eqq-in-context (context)
  (and (not (equal context nil))
       (not (string-equal "openbrac"
                          (car (head-token context))))
          (or (string-equal "=?"
                            (cdr (head-token context)))
              (eqq-in-context (cdr context)))))

(defun name-in-context (name context)
  (and (not (equal context nil))
       (not (string-equal "openbrac"
                          (car (head-token context))))
          (or (string-equal name
                            (cdr (head-token context)))
              (name-in-context name (cdr context)))))


(defun nt ()
  (interactive)
  (message (format "next token: %S" (next-token)))
  )


(defun at-start-of-line (token)
   (save-excursion
       (beginning-of-line)
       (let ((tok (next-token)))
         (string-equal token (cdr tok)))
       )
     )


(defun next-token ()
  (forward-comment (point-max))
  (cond 
   ((looking-at qulog-open-brac-regexp)
    (goto-char (match-end 0))
    (cons "openbrac" (match-string-no-properties 0)))
   ((looking-at qulog-close-brac-regexp)
    (goto-char (match-end 0))
    (cons "closebrac" (match-string-no-properties 0)))
   ((looking-at qulog-name-regexp)
    (goto-char (match-end 0))
    (let ((name (match-string-no-properties 0)))
      (cond 
       ((member name indent-keywords) (cons "indent-operator" name))
       ((member name infix-operators) (cons "infix-operator" name))
       ((member name indent-infix-operators)
        (cons "indent-infix-operator" name))
       ((member name prefix-operators) (cons "prefix-operator" name))
       ((member name quantifier) (cons "quantifier" name))
       ((member name defn-operators) (cons "defn-operator" name))
       (t (cons "name" (match-string-no-properties 0))))
      ))
   ((looking-at qulog-string-regexp)
    ;;(message "string")
    (goto-char (match-end 0))
    (cons "string" (match-string-no-properties 0)))
   ((looking-at qulog-quote-regexp)
    ;(message "quote")
    (goto-char (match-end 0))
    (cons "quoted atom" (match-string-no-properties 0)))
   ((looking-at qulog-number-regexp)
    ;(message "number")
    (goto-char (match-end 0))
    (cons "number" (match-string-no-properties 0)))
   ((looking-at qulog-indent-keywords-regexp)
    ;(message "indent-operators")
    (goto-char (match-end 0))
    (cons "indent-operator" (match-string-no-properties 0)))
   ((looking-at qulog-prefix-operators-regexp)
    (goto-char (match-end 0))
    (cons "prefix-operator" (match-string-no-properties 0)))
   ((looking-at qulog-continuation-operators-regexp)
    ;(message "continuation-operators")
    (goto-char (match-end 0))
    (cons "continuation-operators" (match-string-no-properties 0)))
   ((looking-at qulog-infix-operators-regexp)
    ;(message "infix-operator")
    (goto-char (match-end 0))
    (cons "infix-operator" (match-string-no-properties 0)))
   ((looking-at qulog-indent-infix-operators-regexp)
    ;(message "indent-infix-operator")
    (goto-char (match-end 0))
    (cons "indent-infix-operator" (match-string-no-properties 0)))
   ((looking-at qulog-other-regexp)
    ;(message "other")
    (goto-char (match-end 0))
    (cons "other" (match-string-no-properties 0)))
  
    (t (goto-char (+ (point) 1))
       (cons "default" "default"))
    )
  )

(defun back-to-previous-start ()
  (interactive)
  (while (and (> (point) (point-min))(not (looking-at "^[a-z]")))
    ;;(message (format "start %S" (point)))
    (setq cur (point))
    (forward-comment (- (point)))
    (if (eq cur (point)) (forward-line -1) (beginning-of-line))
    ;;(message (format "end %S %S" (point) cur))
    )

  )

(defun no-more-tokens-on-line ()
  (interactive)
  (save-excursion
    ;;(next-token)
    (let ((start-point (point)))
      (skip-syntax-forward "^<" (line-end-position))
      (forward-comment -1)
      (equal (point) start-point)
      )
    )
  )

(provide 'qulog-mode)
