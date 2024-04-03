FROM gradle:jdk17-jammy@sha256:77a48f339b2bbc261d4448bc5ce7c5aa7c46e1e07b6d8480955aedf6891b7107 as builder

ENV APP_HOME=/usr/app/
WORKDIR $APP_HOME

COPY build.gradle settings.gradle $APP_HOME
COPY gradle $APP_HOME/gradle

RUN gradle build

COPY src $APP_HOME/src

RUN gradle build

RUN mkdir -p out && \
    tar -xvf build/distributions/app.tar -C out

FROM ubuntu:22.04@sha256:77906da86b60585ce12215807090eb327e7386c8fafb5402369e421f44eff17e

LABEL org.opencontainers.image.source https://github.com/Ninjineers-2383/SLAM-Server

RUN apt-get update && apt-get install openjdk-17-jre --no-install-recommends -y && \
    rm -rf /var/lib/apt/lists/*

ENV APP_HOME=/usr/app/
WORKDIR $APP_HOME

COPY --from=builder $APP_HOME/out .

RUN chmod +x ./app/bin/app

CMD ["./app/bin/app"]