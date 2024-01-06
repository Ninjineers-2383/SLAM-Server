FROM gradle:jdk17-jammy

LABEL org.opencontainers.image.source https://github.com/Ninjineers-2383/SLAM-Server

ENV APP_HOME=/usr/app/
WORKDIR $APP_HOME
COPY build.gradle settings.gradle $APP_HOME

COPY gradle $APP_HOME/gradle

COPY src $APP_HOME/src

RUN gradle clean build

CMD ["java", "-jar", "build/libs/SLAM-Server-linuxx64.jar"] 