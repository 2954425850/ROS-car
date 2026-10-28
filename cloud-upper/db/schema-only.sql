--
-- PostgreSQL database dump
--

\restrict MfFQdritVtYjxgKUfSONWyxCrMusEaiYI3bVhU6B6SMwOWam6xgloLFgIUXd7GJ

-- Dumped from database version 14.24 (Ubuntu 14.24-0ubuntu0.22.04.1)
-- Dumped by pg_dump version 14.24 (Ubuntu 14.24-0ubuntu0.22.04.1)

SET statement_timeout = 0;
SET lock_timeout = 0;
SET idle_in_transaction_session_timeout = 0;
SET client_encoding = 'UTF8';
SET standard_conforming_strings = on;
SELECT pg_catalog.set_config('search_path', '', false);
SET check_function_bodies = false;
SET xmloption = content;
SET client_min_messages = warning;
SET row_security = off;

SET default_tablespace = '';

SET default_table_access_method = heap;

--
-- Name: action_recordings; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.action_recordings (
    id bigint NOT NULL,
    name character varying(128) NOT NULL,
    description text,
    duration_ms integer NOT NULL,
    step_count integer NOT NULL,
    created_by bigint,
    created_at timestamp with time zone DEFAULT now() NOT NULL
);


--
-- Name: action_recordings_id_seq; Type: SEQUENCE; Schema: public; Owner: -
--

CREATE SEQUENCE public.action_recordings_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


--
-- Name: action_recordings_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: -
--

ALTER SEQUENCE public.action_recordings_id_seq OWNED BY public.action_recordings.id;


--
-- Name: ai_messages; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.ai_messages (
    id bigint NOT NULL,
    session_id bigint NOT NULL,
    role character varying(16) NOT NULL,
    content text NOT NULL,
    image_ref character varying(256),
    actions jsonb,
    created_at timestamp with time zone DEFAULT now() NOT NULL
);


--
-- Name: ai_messages_id_seq; Type: SEQUENCE; Schema: public; Owner: -
--

CREATE SEQUENCE public.ai_messages_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


--
-- Name: ai_messages_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: -
--

ALTER SEQUENCE public.ai_messages_id_seq OWNED BY public.ai_messages.id;


--
-- Name: ai_sessions; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.ai_sessions (
    id bigint NOT NULL,
    user_id bigint,
    title character varying(128),
    created_at timestamp with time zone DEFAULT now() NOT NULL,
    updated_at timestamp with time zone DEFAULT now() NOT NULL
);


--
-- Name: ai_sessions_id_seq; Type: SEQUENCE; Schema: public; Owner: -
--

CREATE SEQUENCE public.ai_sessions_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


--
-- Name: ai_sessions_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: -
--

ALTER SEQUENCE public.ai_sessions_id_seq OWNED BY public.ai_sessions.id;


--
-- Name: command_logs; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.command_logs (
    id bigint NOT NULL,
    trace_id character varying(64),
    user_id bigint,
    source character varying(16) NOT NULL,
    device character varying(32) NOT NULL,
    payload jsonb NOT NULL,
    accepted boolean NOT NULL,
    error_code integer,
    gateway_source character varying(16) NOT NULL,
    created_at timestamp with time zone DEFAULT now() NOT NULL
);


--
-- Name: command_logs_id_seq; Type: SEQUENCE; Schema: public; Owner: -
--

CREATE SEQUENCE public.command_logs_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


--
-- Name: command_logs_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: -
--

ALTER SEQUENCE public.command_logs_id_seq OWNED BY public.command_logs.id;


--
-- Name: flyway_schema_history; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.flyway_schema_history (
    installed_rank integer NOT NULL,
    version character varying(50),
    description character varying(200) NOT NULL,
    type character varying(20) NOT NULL,
    script character varying(1000) NOT NULL,
    checksum integer,
    installed_by character varying(100) NOT NULL,
    installed_on timestamp without time zone DEFAULT now() NOT NULL,
    execution_time integer NOT NULL,
    success boolean NOT NULL
);


--
-- Name: map_annotations; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.map_annotations (
    id bigint NOT NULL,
    map_id bigint NOT NULL,
    type character varying(16) NOT NULL,
    name character varying(64),
    x double precision NOT NULL,
    y double precision NOT NULL,
    yaw double precision,
    payload jsonb,
    created_at timestamp with time zone DEFAULT now() NOT NULL
);


--
-- Name: map_annotations_id_seq; Type: SEQUENCE; Schema: public; Owner: -
--

CREATE SEQUENCE public.map_annotations_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


--
-- Name: map_annotations_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: -
--

ALTER SEQUENCE public.map_annotations_id_seq OWNED BY public.map_annotations.id;


--
-- Name: maps; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.maps (
    id bigint NOT NULL,
    name character varying(128) NOT NULL,
    version integer NOT NULL,
    data_ref character varying(256) NOT NULL,
    width integer,
    height integer,
    resolution double precision,
    origin_x double precision,
    origin_y double precision,
    created_at timestamp with time zone DEFAULT now() NOT NULL
);


--
-- Name: maps_id_seq; Type: SEQUENCE; Schema: public; Owner: -
--

CREATE SEQUENCE public.maps_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


--
-- Name: maps_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: -
--

ALTER SEQUENCE public.maps_id_seq OWNED BY public.maps.id;


--
-- Name: patrol_points; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.patrol_points (
    route_id bigint NOT NULL,
    seq integer NOT NULL,
    x double precision NOT NULL,
    y double precision NOT NULL,
    yaw double precision
);


--
-- Name: patrol_routes; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.patrol_routes (
    id bigint NOT NULL,
    name character varying(128) NOT NULL,
    loop boolean DEFAULT true NOT NULL,
    created_at timestamp with time zone DEFAULT now() NOT NULL
);


--
-- Name: patrol_routes_id_seq; Type: SEQUENCE; Schema: public; Owner: -
--

CREATE SEQUENCE public.patrol_routes_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


--
-- Name: patrol_routes_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: -
--

ALTER SEQUENCE public.patrol_routes_id_seq OWNED BY public.patrol_routes.id;


--
-- Name: recording_steps; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.recording_steps (
    recording_id bigint NOT NULL,
    seq integer NOT NULL,
    t_ms integer NOT NULL,
    device character varying(32) NOT NULL,
    payload jsonb NOT NULL
);


--
-- Name: revoked_tokens; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.revoked_tokens (
    jti uuid NOT NULL,
    user_id bigint NOT NULL,
    expires_at timestamp with time zone NOT NULL,
    revoked_at timestamp with time zone DEFAULT now() NOT NULL
);


--
-- Name: users; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.users (
    id bigint NOT NULL,
    username character varying(64) NOT NULL,
    password_hash character varying(100) NOT NULL,
    display_name character varying(64),
    role character varying(32) DEFAULT 'ADMIN'::character varying NOT NULL,
    enabled boolean DEFAULT true NOT NULL,
    created_at timestamp with time zone DEFAULT now() NOT NULL,
    updated_at timestamp with time zone DEFAULT now() NOT NULL
);


--
-- Name: users_id_seq; Type: SEQUENCE; Schema: public; Owner: -
--

CREATE SEQUENCE public.users_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


--
-- Name: users_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: -
--

ALTER SEQUENCE public.users_id_seq OWNED BY public.users.id;


--
-- Name: video_clips; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.video_clips (
    id bigint NOT NULL,
    kind character varying(16) NOT NULL,
    storage character varying(16) NOT NULL,
    object_key character varying(256) NOT NULL,
    size_bytes bigint NOT NULL,
    duration_ms integer,
    started_at timestamp with time zone NOT NULL,
    created_by bigint,
    created_at timestamp with time zone DEFAULT now() NOT NULL
);


--
-- Name: video_clips_id_seq; Type: SEQUENCE; Schema: public; Owner: -
--

CREATE SEQUENCE public.video_clips_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


--
-- Name: video_clips_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: -
--

ALTER SEQUENCE public.video_clips_id_seq OWNED BY public.video_clips.id;


--
-- Name: action_recordings id; Type: DEFAULT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.action_recordings ALTER COLUMN id SET DEFAULT nextval('public.action_recordings_id_seq'::regclass);


--
-- Name: ai_messages id; Type: DEFAULT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.ai_messages ALTER COLUMN id SET DEFAULT nextval('public.ai_messages_id_seq'::regclass);


--
-- Name: ai_sessions id; Type: DEFAULT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.ai_sessions ALTER COLUMN id SET DEFAULT nextval('public.ai_sessions_id_seq'::regclass);


--
-- Name: command_logs id; Type: DEFAULT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.command_logs ALTER COLUMN id SET DEFAULT nextval('public.command_logs_id_seq'::regclass);


--
-- Name: map_annotations id; Type: DEFAULT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.map_annotations ALTER COLUMN id SET DEFAULT nextval('public.map_annotations_id_seq'::regclass);


--
-- Name: maps id; Type: DEFAULT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.maps ALTER COLUMN id SET DEFAULT nextval('public.maps_id_seq'::regclass);


--
-- Name: patrol_routes id; Type: DEFAULT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.patrol_routes ALTER COLUMN id SET DEFAULT nextval('public.patrol_routes_id_seq'::regclass);


--
-- Name: users id; Type: DEFAULT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.users ALTER COLUMN id SET DEFAULT nextval('public.users_id_seq'::regclass);


--
-- Name: video_clips id; Type: DEFAULT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.video_clips ALTER COLUMN id SET DEFAULT nextval('public.video_clips_id_seq'::regclass);


--
-- Name: action_recordings action_recordings_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.action_recordings
    ADD CONSTRAINT action_recordings_pkey PRIMARY KEY (id);


--
-- Name: ai_messages ai_messages_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.ai_messages
    ADD CONSTRAINT ai_messages_pkey PRIMARY KEY (id);


--
-- Name: ai_sessions ai_sessions_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.ai_sessions
    ADD CONSTRAINT ai_sessions_pkey PRIMARY KEY (id);


--
-- Name: command_logs command_logs_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.command_logs
    ADD CONSTRAINT command_logs_pkey PRIMARY KEY (id);


--
-- Name: flyway_schema_history flyway_schema_history_pk; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.flyway_schema_history
    ADD CONSTRAINT flyway_schema_history_pk PRIMARY KEY (installed_rank);


--
-- Name: map_annotations map_annotations_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.map_annotations
    ADD CONSTRAINT map_annotations_pkey PRIMARY KEY (id);


--
-- Name: maps maps_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.maps
    ADD CONSTRAINT maps_pkey PRIMARY KEY (id);


--
-- Name: patrol_points patrol_points_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.patrol_points
    ADD CONSTRAINT patrol_points_pkey PRIMARY KEY (route_id, seq);


--
-- Name: patrol_routes patrol_routes_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.patrol_routes
    ADD CONSTRAINT patrol_routes_pkey PRIMARY KEY (id);


--
-- Name: recording_steps recording_steps_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.recording_steps
    ADD CONSTRAINT recording_steps_pkey PRIMARY KEY (recording_id, seq);


--
-- Name: revoked_tokens revoked_tokens_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.revoked_tokens
    ADD CONSTRAINT revoked_tokens_pkey PRIMARY KEY (jti);


--
-- Name: maps uk_maps_name_version; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.maps
    ADD CONSTRAINT uk_maps_name_version UNIQUE (name, version);


--
-- Name: users uk_users_username; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.users
    ADD CONSTRAINT uk_users_username UNIQUE (username);


--
-- Name: users users_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.users
    ADD CONSTRAINT users_pkey PRIMARY KEY (id);


--
-- Name: video_clips video_clips_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.video_clips
    ADD CONSTRAINT video_clips_pkey PRIMARY KEY (id);


--
-- Name: flyway_schema_history_s_idx; Type: INDEX; Schema: public; Owner: -
--

CREATE INDEX flyway_schema_history_s_idx ON public.flyway_schema_history USING btree (success);


--
-- Name: idx_aimsg_session; Type: INDEX; Schema: public; Owner: -
--

CREATE INDEX idx_aimsg_session ON public.ai_messages USING btree (session_id, created_at);


--
-- Name: idx_cmdlogs_created; Type: INDEX; Schema: public; Owner: -
--

CREATE INDEX idx_cmdlogs_created ON public.command_logs USING btree (created_at DESC);


--
-- Name: idx_cmdlogs_device; Type: INDEX; Schema: public; Owner: -
--

CREATE INDEX idx_cmdlogs_device ON public.command_logs USING btree (device, created_at DESC);


--
-- Name: idx_cmdlogs_user; Type: INDEX; Schema: public; Owner: -
--

CREATE INDEX idx_cmdlogs_user ON public.command_logs USING btree (user_id, created_at DESC);


--
-- Name: idx_mapanno_map; Type: INDEX; Schema: public; Owner: -
--

CREATE INDEX idx_mapanno_map ON public.map_annotations USING btree (map_id, type);


--
-- Name: idx_recordings_created; Type: INDEX; Schema: public; Owner: -
--

CREATE INDEX idx_recordings_created ON public.action_recordings USING btree (created_at DESC);


--
-- Name: idx_revoked_expires; Type: INDEX; Schema: public; Owner: -
--

CREATE INDEX idx_revoked_expires ON public.revoked_tokens USING btree (expires_at);


--
-- Name: idx_videoclips_started; Type: INDEX; Schema: public; Owner: -
--

CREATE INDEX idx_videoclips_started ON public.video_clips USING btree (started_at DESC);


--
-- Name: action_recordings action_recordings_created_by_fkey; Type: FK CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.action_recordings
    ADD CONSTRAINT action_recordings_created_by_fkey FOREIGN KEY (created_by) REFERENCES public.users(id) ON DELETE SET NULL;


--
-- Name: ai_messages ai_messages_session_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.ai_messages
    ADD CONSTRAINT ai_messages_session_id_fkey FOREIGN KEY (session_id) REFERENCES public.ai_sessions(id) ON DELETE CASCADE;


--
-- Name: ai_sessions ai_sessions_user_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.ai_sessions
    ADD CONSTRAINT ai_sessions_user_id_fkey FOREIGN KEY (user_id) REFERENCES public.users(id) ON DELETE CASCADE;


--
-- Name: command_logs command_logs_user_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.command_logs
    ADD CONSTRAINT command_logs_user_id_fkey FOREIGN KEY (user_id) REFERENCES public.users(id) ON DELETE SET NULL;


--
-- Name: map_annotations map_annotations_map_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.map_annotations
    ADD CONSTRAINT map_annotations_map_id_fkey FOREIGN KEY (map_id) REFERENCES public.maps(id) ON DELETE CASCADE;


--
-- Name: patrol_points patrol_points_route_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.patrol_points
    ADD CONSTRAINT patrol_points_route_id_fkey FOREIGN KEY (route_id) REFERENCES public.patrol_routes(id) ON DELETE CASCADE;


--
-- Name: recording_steps recording_steps_recording_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.recording_steps
    ADD CONSTRAINT recording_steps_recording_id_fkey FOREIGN KEY (recording_id) REFERENCES public.action_recordings(id) ON DELETE CASCADE;


--
-- Name: revoked_tokens revoked_tokens_user_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.revoked_tokens
    ADD CONSTRAINT revoked_tokens_user_id_fkey FOREIGN KEY (user_id) REFERENCES public.users(id) ON DELETE CASCADE;


--
-- Name: video_clips video_clips_created_by_fkey; Type: FK CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.video_clips
    ADD CONSTRAINT video_clips_created_by_fkey FOREIGN KEY (created_by) REFERENCES public.users(id) ON DELETE SET NULL;


--
-- PostgreSQL database dump complete
--

\unrestrict MfFQdritVtYjxgKUfSONWyxCrMusEaiYI3bVhU6B6SMwOWam6xgloLFgIUXd7GJ

