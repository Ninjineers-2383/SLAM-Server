FROM gradle:jdk17-alpine AS BUILD
ENV APP_HOME=/usr/app/
WORKDIR $APP_HOME
COPY build.gradle settings.gradle $APP_HOME

COPY gradle $APP_HOME/gradle
COPY --chown=gradle:gradle . /home/gradle/src
USER root
RUN chown -R gradle /home/gradle/src

RUN gradle build || return 0
COPY . .
RUN gradle clean build

# actual container
FROM openjdk:17-slim-buster
ENV ARTIFACT_NAME=SLAM-Server
ENV APP_HOME=/usr/app/

WORKDIR $APP_HOME
COPY --from=BUILD $APP_HOME/build/distributions/${ARTIFACT_NAME}.tar .
RUN tar -xf ${ARTIFACT_NAME}.tar && rm ${ARTIFACT_NAME}.tar

RUN chmod +x ${APP_HOME}/${ARTIFACT_NAME}/bin/${ARTIFACT_NAME}

CMD /${APP_HOME}/${ARTIFACT_NAME}/bin/${ARTIFACT_NAME}