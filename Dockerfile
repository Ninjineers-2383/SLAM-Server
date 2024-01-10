FROM gradle:jdk17-jammy as builder

ENV APP_HOME=/usr/app/
WORKDIR $APP_HOME
COPY build.gradle settings.gradle $APP_HOME

COPY gradle $APP_HOME/gradle

COPY src $APP_HOME/src

RUN gradle clean build

RUN mkdir -p out && \
    tar -xvf build/distributions/app.tar -C out

FROM ubuntu:22.04

LABEL org.opencontainers.image.source https://github.com/Ninjineers-2383/SLAM-Server

RUN apt-get update && apt-get install openjdk-17-jre --no-install-recommends -y && \
    rm -rf /var/lib/apt/lists/*

ENV APP_HOME=/usr/app/
WORKDIR $APP_HOME

COPY --from=builder $APP_HOME/out .

RUN chmod +x ./app/bin/app

CMD ["./app/bin/app"]