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

  ;; IHコンロの位置を触覚で確認
  ;; *ih-knob-coords*の値を書き換えるので、注意
  (correct-ih-knob-coords)

  ;; センサ貼付け位置を探し、貼る
  (place-sensor)

  ;; 沸騰させる
  (boil-soup)

  ;; お湯を注ぐ
  (pour-soup)

  ;; 冷ます
  (cool-soup)

  ;; センサを取り外す
  nil
  )