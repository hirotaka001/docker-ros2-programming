# 基本となるイメージ
FROM ubuntu:18.04
#　ユーザの情報
LABEL maintainer = "Hiroaki Takahashi <hrktkhsh63784@gmail.com>"
ENV TZ JST-9
SHELL ["/bin/bash", "-c"]

# 必要なパッケージのインストール
RUN apt-get update \
&& apt-get install -y python3-pip python3-dev \
&& cd /usr/local/bin \
&& ln -s /usr/bin/python3 python \
&& pip3 install --upgrade pip

# コンテナ十実行時にpython3を実行する
ENTRYPOINT ["python3"]