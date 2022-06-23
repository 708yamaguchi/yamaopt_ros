(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/arrangement-at-arrange.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/ih-at-arrange.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/motion/tool-at-arrange.l")

(load "package://yamaopt_ros/demo/utils.l")

;; 準備：最初に換気扇とIHの電源を入れていく
(ros::roseus "soup_from_boil" :anonymous t)

(defun setup ()
  ;; 最初の位置はarrangeの場所
  (move-to-arrange-with-cable-check)

  ;; IHコンロの位置を触覚で確認
  ;; *ih-knob-coords*の値を書き換えるので、注意
  (correct-ih-knob-coords)

  ;; IHコンロの操作チェック
  (ih-check)

  ;; 設置するもの：おたまとお皿
  ;; 人からおたまを受け取る
  (receive-object-with-dialogue :object :ladle)
  ;; 人に頼んで、コップ（皿）をキッチンに置いてもらう
  (set-plate-with-dialogue)
  ;; main()で使うために、おたまをキッチンの上に置く
  ;; (set-ladle-at-arrange)
  (set-ladle-at-kitchen)
  )

(defun main ()
  ;; IHコンロの前に移動（充電ケーブルは常に抜いた状態）
  (move-real-robot :to :arrange)

  ;; 周囲を見てセンサ配置を探し、貼り付ける
  ;; 元々のsoup-from-boilのデモでは、*pr2*は2号館座標系に存在するが、
  ;; (place-sensor)では*pr2*が原点の座標系で記述されているので、一時的に座標系を変えている。
  ;; TODO: angle vectorの中継地点を上手に設定しないと、鍋の取っ手に手が引っかかる
  ;; TODO: roslaunch から立ち上げる場合は、~armというrosparamをセットするので、引数は必要ない。
  (let ((demo-coords (send *pr2* :copy-worldcoords)))
    (send *pr2* :move-to (make-coords) :world)
    (place-sensor "left")
    (send *pr2* :move-to demo-coords :world))

  ;; IHコンロの位置を触覚で確認
  ;; *ih-knob-coords*の値を書き換えるので、注意
  (correct-ih-knob-coords)

  ;; 沸騰させる
  (boil-soup)

  ;; お湯を注ぐ
  (pour-soup)
  )
