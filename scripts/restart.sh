#!/bin/bash
# restart.bash
# 以下のコマンドで実行権限を与えてください
#   $ chmod a+x restart.bash
# 使い方
#   $   ./restart.bash 
#   ex-restart) ./restart.bash date -h
#   ex-end    ) ./restart.bash date --help
if test $# -le 0  ; then
	echo "USAGE: command"
	exit -1
fi

ret=-1

# ret が 0のとき以外、プログラムを再起動し続ける
while test ${ret} -ne 0
do
	echo "----> start: $*"
	# プログラムの実行
	$@
	
	# 戻り値をチェック
	ret=$?
	echo "----< end : ret = $ret"
	
	# 再起動するまでにちょっとスリープ
	# これが無いと、結構困る
	sleep 1;
done

echo "end"
